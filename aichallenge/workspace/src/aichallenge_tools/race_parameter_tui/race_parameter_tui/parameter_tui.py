#!/usr/bin/env python3
"""Curses UI for runtime tuning of the race controller."""

from __future__ import annotations

import curses
import threading
import time
from typing import Any

from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from .logic import (
    allowed_parameter_names,
    discover_candidate_nodes,
    format_parameter_value,
    NodeCandidate,
    ParameterEntry,
    parse_parameter_text,
    type_name_from_parameter_value,
    value_from_parameter_value,
    visible_parameter_slice,
)


class AsyncParameterClient:
    """Small Humble-compatible async parameter service client.

    ``rclpy.parameter_client`` is available only in newer ROS 2 releases;
    Humble exposes the same services but not that convenience wrapper.
    """

    def __init__(self, node: Node, remote_node_name: str, remote_node_namespace: str) -> None:
        namespace = remote_node_namespace.rstrip('/')
        prefix = f'{namespace}/{remote_node_name}' if namespace else f'/{remote_node_name}'
        self._list_client = node.create_client(ListParameters, f'{prefix}/list_parameters')
        self._get_client = node.create_client(GetParameters, f'{prefix}/get_parameters')
        self._set_client = node.create_client(SetParameters, f'{prefix}/set_parameters')

    def list_parameters(self, prefixes: list[str], depth: int) -> Any:
        request = ListParameters.Request()
        request.prefixes = prefixes
        request.depth = depth
        return self._list_client.call_async(request)

    def get_parameters(self, names: list[str]) -> Any:
        request = GetParameters.Request()
        request.names = names
        return self._get_client.call_async(request)

    def set_parameters(self, parameters: list[Parameter]) -> Any:
        request = SetParameters.Request()
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        return self._set_client.call_async(request)


class ParameterTuiNode(Node):
    """ROS bridge; all service calls are completed while the executor spins."""

    def __init__(self) -> None:
        super().__init__('race_parameter_tui')
        self._parameter_clients: dict[str, AsyncParameterClient] = {}

    @staticmethod
    def _wait_for_future(future: Any, timeout_sec: float = 2.0) -> Any:
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise TimeoutError('parameter service timed out')
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(str(exception)) from exception
        return future.result()

    def _client(self, candidate: NodeCandidate) -> AsyncParameterClient:
        client = self._parameter_clients.get(candidate.full_name)
        if client is None:
            client = AsyncParameterClient(self, candidate.name, candidate.namespace)
            self._parameter_clients[candidate.full_name] = client
        return client

    def discover(self) -> list[NodeCandidate]:
        return discover_candidate_nodes(self.get_node_names_and_namespaces())

    def read_parameters(self, candidate: NodeCandidate) -> list[ParameterEntry]:
        client = self._client(candidate)
        response = self._wait_for_future(client.list_parameters([], 0))
        available = list(response.result.names)
        names = allowed_parameter_names(candidate.name, available)
        if not names:
            return []
        response = self._wait_for_future(client.get_parameters(names))
        entries: list[ParameterEntry] = []
        for name, value in zip(names, response.values):
            type_name = type_name_from_parameter_value(value)
            if type_name is None:
                continue
            entries.append(ParameterEntry(name, type_name, value_from_parameter_value(value)))
        return entries

    def set_parameter(self, candidate: NodeCandidate, entry: ParameterEntry, value: object) -> str:
        client = self._client(candidate)
        response = self._wait_for_future(
            client.set_parameters([Parameter(entry.name, value=value)])
        )
        results = getattr(response, 'results', response)
        if not results:
            return 'remote node returned no result'
        result = results[0]
        if not result.successful:
            return result.reason or 'remote node rejected the value'
        return 'updated'


class ParameterTui:
    """Curses state and input handling for the currently selected node."""

    def __init__(self, node: ParameterTuiNode) -> None:
        self.node = node
        self.candidates: list[NodeCandidate] = []
        self.selected_node = 0
        self.entries: list[ParameterEntry] = []
        self.selected_parameter = 0
        self.scroll_offset = 0
        self.status = 'Press r to discover controller nodes.'

    @property
    def candidate(self) -> NodeCandidate | None:
        if not self.candidates:
            return None
        return self.candidates[self.selected_node]

    def refresh(self) -> None:
        try:
            old_name = self.candidate.full_name if self.candidate else None
            self.candidates = self.node.discover()
            if old_name:
                self.selected_node = next(
                    (index for index, candidate in enumerate(self.candidates)
                     if candidate.full_name == old_name),
                    min(self.selected_node, max(len(self.candidates) - 1, 0)),
                )
            else:
                self.selected_node = min(self.selected_node, max(len(self.candidates) - 1, 0))
            self.selected_parameter = 0
            self.scroll_offset = 0
            self._read_selected_parameters()
            if not self.candidates:
                self.status = 'No candidate nodes found.'
        except Exception as exc:  # noqa: BLE001 - UI must stay usable on graph errors.
            self.entries = []
            self.status = f'refresh failed: {exc}'

    def _read_selected_parameters(self) -> None:
        candidate = self.candidate
        if candidate is None:
            self.entries = []
            self.scroll_offset = 0
            return
        try:
            self.entries = self.node.read_parameters(candidate)
            self.selected_parameter = min(
                self.selected_parameter, max(len(self.entries) - 1, 0)
            )
            self.scroll_offset = 0
            self.status = f'{candidate.full_name}: {len(self.entries)} supported parameter(s)'
        except Exception as exc:  # noqa: BLE001 - report service failures in the status line.
            self.entries = []
            self.scroll_offset = 0
            self.status = f'read failed: {exc}'

    def move_parameter(self, delta: int) -> None:
        if self.entries:
            self.selected_parameter = max(
                0, min(self.selected_parameter + delta, len(self.entries) - 1)
            )

    def move_node(self, delta: int) -> None:
        if not self.candidates:
            return
        self.selected_node = (self.selected_node + delta) % len(self.candidates)
        self.selected_parameter = 0
        self.scroll_offset = 0
        self._read_selected_parameters()

    def edit_selected(self, screen: Any) -> None:
        candidate = self.candidate
        if candidate is None or not self.entries:
            self.status = 'No editable parameter is selected.'
            return
        entry = self.entries[self.selected_parameter]
        if entry.type_name == 'bool':
            value = not bool(entry.value)
        else:
            prompt = f'{entry.name} [{format_parameter_value(entry.value, entry.type_name)}]: '
            try:
                curses.echo()
                screen.nodelay(False)
                screen.addstr(max(curses.LINES - 2, 0), 0, prompt[: max(curses.COLS - 1, 1)])
                screen.clrtoeol()
                raw = screen.getstr(
                    max(curses.LINES - 2, 0), len(prompt),
                    max(curses.COLS - len(prompt) - 1, 1),
                )
                text = raw.decode(errors='replace')
            finally:
                curses.noecho()
                screen.nodelay(True)
            if text == '':
                self.status = 'edit cancelled'
                return
            try:
                value = parse_parameter_text(text, entry.type_name)
            except ValueError as exc:
                self.status = f'invalid value: {exc}'
                return
        try:
            reason = self.node.set_parameter(candidate, entry, value)
            if reason == 'updated':
                entry.value = value
                self.status = f'{entry.name}: updated'
            else:
                self.status = f'{entry.name}: rejected ({reason})'
        except Exception as exc:  # noqa: BLE001 - keep curses alive after service failures.
            self.status = f'set failed: {exc}'

    def draw(self, screen: Any) -> None:
        screen.erase()
        screen.addstr(
            0, 0, 'Race parameter TUI  (q quit, r refresh, Tab node, Enter edit)',
            curses.A_BOLD,
        )
        if self.candidates:
            node_text = ' | '.join(
                f'[{candidate.full_name}]' if index == self.selected_node else candidate.full_name
                for index, candidate in enumerate(self.candidates)
            )
            screen.addnstr(1, 0, node_text, max(curses.COLS - 1, 1))
        else:
            screen.addstr(1, 0, '(no candidate nodes)')

        viewport_size = max(curses.LINES - 6, 1)
        start, end = visible_parameter_slice(
            len(self.entries),
            self.selected_parameter,
            viewport_size,
            self.scroll_offset,
        )
        self.scroll_offset = start
        if self.entries and curses.LINES > 3:
            direction = ''
            if start > 0:
                direction += '↑ '
            if end < len(self.entries):
                direction += '↓ '
            screen.addnstr(
                2, 0,
                f'{direction}parameters {start + 1}-{end}/{len(self.entries)}',
                max(curses.COLS - 1, 1),
            )
        if not self.entries and self.candidate:
            screen.addnstr(
                3, 0, 'No allowlisted scalar parameters declared on this node.',
                max(curses.COLS - 1, 1),
            )
        for index in range(start, end):
            entry = self.entries[index]
            row = index - start + 3
            if row >= curses.LINES - 3:
                break
            marker = '>' if index == self.selected_parameter else ' '
            value = format_parameter_value(entry.value, entry.type_name)
            text = f'{marker} {entry.name:<34} {value:<18} ({entry.type_name})'
            try:
                screen.addnstr(
                    row, 0, text, max(curses.COLS - 1, 1),
                    curses.A_REVERSE if index == self.selected_parameter else 0,
                )
            except curses.error:
                pass
        screen.addnstr(max(curses.LINES - 2, 0), 0, self.status, max(curses.COLS - 1, 1))
        screen.addnstr(
            max(curses.LINES - 1, 0), 0,
            '↑/↓ select  PgUp/PgDn node  r refresh  Enter edit/toggle  q quit',
            max(curses.COLS - 1, 1),
        )
        screen.refresh()

    def run(self, screen: Any) -> None:
        curses.curs_set(0)
        screen.keypad(True)
        screen.nodelay(True)
        self.refresh()
        while True:
            self.draw(screen)
            key = screen.getch()
            if key in (ord('q'), ord('Q'), 27):
                return
            if key in (ord('r'), ord('R')):
                self.refresh()
            elif key in (curses.KEY_UP, ord('k')):
                self.move_parameter(-1)
            elif key in (curses.KEY_DOWN, ord('j')):
                self.move_parameter(1)
            elif key in (curses.KEY_PPAGE, ord('[')):
                self.move_node(-1)
            elif key in (curses.KEY_NPAGE, ord(']'), 9):
                self.move_node(1)
            elif key in (curses.KEY_ENTER, 10, 13):
                self.edit_selected(screen)
            time.sleep(0.03)


def main() -> None:
    rclpy.init()
    node = ParameterTuiNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        curses.wrapper(ParameterTui(node).run)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()

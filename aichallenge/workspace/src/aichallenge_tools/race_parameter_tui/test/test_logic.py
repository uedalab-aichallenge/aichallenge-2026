from types import SimpleNamespace

import pytest

from race_parameter_tui.logic import (
    allowed_parameter_names,
    discover_candidate_nodes,
    parse_parameter_text,
    type_name_from_parameter_value,
    value_from_parameter_value,
    visible_parameter_slice,
)


def test_discovery_preserves_namespace_and_filters_candidates():
    nodes = discover_candidate_nodes(
        [
            ('simple_pure_pursuit', '/vehicle_2'),
            ('unrelated', '/vehicle_2'),
            ('reverse_pure_pursuit', '/'),
            ('simple_pure_pursuit', '/vehicle_1'),
        ]
    )
    assert [node.full_name for node in nodes] == [
        '/reverse_pure_pursuit',
        '/vehicle_1/simple_pure_pursuit',
        '/vehicle_2/simple_pure_pursuit',
    ]


def test_allowlist_intersects_declared_parameters():
    assert allowed_parameter_names(
        'simple_pure_pursuit',
        ['lookahead_gain', 'use_external_target_vel', 'use_sim_time', 'array'],
    ) == ['lookahead_gain', 'use_external_target_vel']


@pytest.mark.parametrize(
    ('text', 'type_name', 'expected'),
    [
        ('true', 'bool', True),
        ('OFF', 'bool', False),
        ('-12', 'int', -12),
        ('1.25', 'double', 1.25),
        ('165:185:0.35', 'string', '165:185:0.35'),
    ],
)
def test_parse_parameter_text(text, type_name, expected):
    assert parse_parameter_text(text, type_name) == expected


@pytest.mark.parametrize(
    ('text', 'type_name'),
    [('maybe', 'bool'), ('1.2', 'int'), ('nan', 'double'), ('1e309', 'double')],
)
def test_parse_parameter_text_rejects_unsafe_values(text, type_name):
    with pytest.raises(ValueError):
        parse_parameter_text(text, type_name)


def test_parameter_value_conversion_for_supported_scalar_types():
    value = SimpleNamespace(type=3, double_value=2.5)
    assert type_name_from_parameter_value(value) == 'double'
    assert value_from_parameter_value(value) == 2.5


@pytest.mark.parametrize(
    ('item_count', 'selected_index', 'viewport_size', 'scroll_offset', 'expected'),
    [
        (0, 0, 5, 0, (0, 0)),
        (3, 1, 5, 0, (0, 3)),
        (20, 0, 5, 0, (0, 5)),
        (20, 5, 5, 0, (1, 6)),
        (20, 19, 5, 0, (15, 20)),
        (20, 4, 5, 8, (4, 9)),
        (20, 2, 5, 99, (2, 7)),
    ],
)
def test_visible_parameter_slice_keeps_selected_item_visible(
    item_count, selected_index, viewport_size, scroll_offset, expected
):
    assert visible_parameter_slice(
        item_count, selected_index, viewport_size, scroll_offset
    ) == expected

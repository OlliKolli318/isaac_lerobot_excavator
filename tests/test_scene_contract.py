"""Check embedded Python and safety behavior without opening Isaac or USD."""
import ast
from pathlib import Path
import re
from types import SimpleNamespace as NS

import pytest

SCENE = Path(__file__).resolve().parents[1] / 'env.usd'


def scripts():
    pattern = r'string inputs:script = (""".*?"""|"(?:[^"\\]|\\.)*")'
    return [ast.literal_eval(match.group(1)) for match in re.finditer(pattern, SCENE.read_text(), re.S)]


def test_embedded_scripts_compile():
    blocks = scripts()
    assert len(blocks) == 9
    for index, script in enumerate(blocks):
        compile(script, f'scene_script_{index}', 'exec')


@pytest.mark.parametrize('optimized', [0, 2])
@pytest.mark.parametrize('control', ['0:0:autonomous', '1:-1:autonomous',
                                   f'{2**52}:0:autonomous', '1:0:unknown', 'malformed'])
def test_invalid_dagger_request_holds_even_with_python_optimization(optimized, control):
    namespace = {}
    exec(compile(scripts()[0], 'action_mux', 'exec', optimize=optimized), namespace)
    state = NS()
    db = NS(per_instance_state=state, outputs=NS(), inputs=NS(
        daggerState=control, daggerLeftCount=0, daggerRightCount=0,
        daggerTriggersReleased=True, controlEnabled=True))
    namespace['setup'](db)
    phase, enabled = namespace['splat_dagger_select_phase'](db, state)
    assert phase == 'idle'
    assert not enabled
    assert not db.outputs.effectiveEnabled


def test_new_phase_discards_queued_actions_and_preserves_claw():
    namespace = {}
    exec(compile(scripts()[0], 'action_mux', 'exec'), namespace)
    state = NS()
    db = NS(per_instance_state=state, outputs=NS(), inputs=NS(
        daggerState='1:0:autonomous', daggerLeftCount=0, daggerRightCount=0,
        daggerTriggersReleased=True, controlEnabled=True))
    namespace['setup'](db)
    assert namespace['splat_dagger_select_phase'](db, state) == ('autonomous', True)
    state.pending_actions[100] = [1, 0, 0, 0, 0]
    state.last_command_ns = 100
    state.splat_dagger_claw = 0
    db.inputs.daggerState = '1:1:paused'
    assert namespace['splat_dagger_select_phase'](db, state) == ('paused', False)
    assert not state.pending_actions
    assert state.last_command_ns == -1
    assert state.last_policy_action == [0, 0, 0, 0, 0]


def test_local_scene_references_exist():
    references = re.findall(r'@([^@\r\n]+)@', SCENE.read_text())
    assert references
    for reference in references:
        assert '://' not in reference
        if reference == 'OmniGlass.mdl':
            # This shader module is resolved by Isaac's MDL search path.
            continue
        assert (SCENE.parent / reference).is_file()

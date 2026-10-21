from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from simulation_transport import DEFAULT_PROFILE, configure_simulation_transport


def test_default_profile_and_alias_cleanup():
    environ = {'FASTRTPS_DEFAULT_PROFILES_FILE': 'old.xml'}
    assert configure_simulation_transport(environ) == DEFAULT_PROFILE
    assert environ == {'FASTDDS_DEFAULT_PROFILES_FILE': str(DEFAULT_PROFILE)}
    root = ET.parse(DEFAULT_PROFILE).getroot()
    ns = {'dds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    assert root.find('.//dds:type', ns).text == 'UDPv4'


@pytest.mark.parametrize('override', [
    {'KAIVURI_FASTDDS_PROFILE': ' InHerIt '}, {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}])
def test_inherited_or_other_transport_is_unchanged(override):
    environ = {'FASTRTPS_DEFAULT_PROFILES_FILE': 'local.xml', **override}
    before = environ.copy()
    assert configure_simulation_transport(environ) is None
    assert environ == before


def test_missing_override_does_not_change_environment(tmp_path):
    environ = {'KAIVURI_FASTDDS_PROFILE': str(tmp_path / 'missing.xml')}
    before = environ.copy()
    with pytest.raises(FileNotFoundError):
        configure_simulation_transport(environ)
    assert environ == before


def test_explicit_profile(tmp_path):
    profile = tmp_path / 'profile.xml'
    profile.touch()
    environ = {'KAIVURI_FASTDDS_PROFILE': str(profile)}
    assert configure_simulation_transport(environ) == Path(profile).resolve()

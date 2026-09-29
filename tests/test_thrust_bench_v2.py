"""Electrical-speed/DShot-current records cannot silently consume v1 RPM."""
import json
from dataclasses import fields

import pytest

from tools.thrust_bench.current_sources import dshot_total_current, dshot_input_power
from tools.thrust_bench.records import BenchSample, SCHEMA_VERSION
from tools.thrust_bench.session import SessionStore, read_samples


def test_v2_records_preserve_erpm_esc_current_and_separate_board_diagnostics(tmp_path):
    sample=BenchSample("r", "s", 1.0, upper_erpm=14000, lower_erpm=28000,
        upper_esc_current_a=2, lower_esc_current_a=3,
        upper_esc_current_age_ms=10, lower_esc_current_age_ms=15,
        board_current_a=999, board_current_calibrated=True)
    store=SessionStore({"schema_version":1,"speed_domain":"electrical_erpm","current_source":"dshot"},tmp_path)
    store.sample(sample)
    assert SCHEMA_VERSION==2
    assert json.loads((store.path/"metadata.json").read_text(encoding="utf-8"))["schema_version"]==2
    restored=read_samples(store.samples_path)[0]
    assert restored==sample
    assert restored.upper_erpm==14000 and restored.lower_erpm==28000
    assert restored.board_current_a==999 and restored.esc_current_calibrated is False
    assert "upper_rpm" not in {f.name for f in fields(BenchSample)}


def test_old_mechanical_rpm_csv_and_dict_are_explicitly_rejected(tmp_path):
    old=tmp_path/"old.csv"
    old.write_text("schema_version,run_id,segment_id,host_time_s,upper_rpm,lower_rpm\n1,r,s,0,2000,4000\n",encoding="utf-8")
    with pytest.raises(ValueError,match="机械转速"):
        read_samples(old)
    with pytest.raises(ValueError,match="mechanical RPM"):
        BenchSample.from_dict({"schema_version":1,"run_id":"r","segment_id":"s","host_time_s":0})


@pytest.mark.parametrize("values", [(None,3,10,10),(2,None,10,10),(2,3,None,10),
    (2,3,1001,1001),(2,3,0,51),(float('nan'),3,0,0),(-1,3,0,0),(2,3,-1,0)])
def test_missing_invalid_stale_or_unaligned_dshot_pair_has_no_total(values):
    assert dshot_total_current(*values) is None
    assert dshot_input_power(12,0,*values) is None


def test_pair_power_uses_dshot_sum_and_voltage_alignment_including_real_zero():
    assert dshot_total_current(2,3,10,20)==5
    assert dshot_input_power(12,15,2,3,10,20)==60
    assert dshot_input_power(12,80,2,3,10,20) is None
    assert dshot_input_power(12,300,2,3,300,300) is None
    assert dshot_input_power(12,0,0,0,0,0)==0

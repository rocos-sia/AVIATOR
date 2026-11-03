import json
import pytest
from tools.gfr1_resume_unseen_regression import load_completed


def test_completed_case_requires_consistent_immutable_result(tmp_path):
    row={'episode':0,'method':'original','progress':.5}
    case=tmp_path/'episode_00_original';case.mkdir()
    (case/'result.json').write_text(json.dumps(row))
    (tmp_path/'episodes.jsonl').write_text(json.dumps(row)+'\n')
    rows, keys=load_completed(tmp_path)
    assert rows==[row] and keys=={(0,'original')}
    (case/'result.json').write_text(json.dumps(dict(row,progress=.6)))
    with pytest.raises(ValueError,match='disagree'):
        load_completed(tmp_path)


def test_duplicate_finished_case_not_counted_twice(tmp_path):
    row={'episode':0,'method':'combined'}
    case=tmp_path/'episode_00_combined';case.mkdir()
    (case/'result.json').write_text(json.dumps(row))
    (tmp_path/'episodes.jsonl').write_text((json.dumps(row)+'\n')*2)
    with pytest.raises(ValueError,match='duplicate'):
        load_completed(tmp_path)


def test_unlogged_completed_result_is_not_overwritten(tmp_path):
    case=tmp_path/'episode_00_combined';case.mkdir()
    (case/'result.json').write_text(json.dumps({'episode':0,'method':'combined'}))
    (tmp_path/'episodes.jsonl').write_text('')
    with pytest.raises(ValueError,match='orphan'):
        load_completed(tmp_path)

import json

from tools.cad.rigid_body_fixture import _load_points


def test_load_points_reads_custom_rigids_in_meters(tmp_path):
    path = tmp_path / "candidate.json"
    path.write_text(
        json.dumps(
            {
                "custom_rigids": [
                    {
                        "name": "head",
                        "marker_positions": [
                            [0.001, 0.002, 0.003],
                            [0.004, 0.005, 0.006],
                            [0.007, 0.008, 0.009],
                        ],
                        "marker_diameter_m": 0.014,
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    assert _load_points(path, "head") == [
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
        (7.0, 8.0, 9.0),
    ]

from seedling_semantic_mapping.color_sep_localizer import ColorSepLocalizer
from seedling_semantic_mapping.yolo_sep_localizer import YoloSepLocalizer


def test_yolo_localizer_loads_model():
    assert YoloSepLocalizer.uses_yolo_model(None)


def test_color_localizer_skips_yolo_model():
    assert not ColorSepLocalizer.uses_yolo_model(None)

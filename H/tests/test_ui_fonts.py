import sys
import unittest
from pathlib import Path


H_DIR = Path(__file__).resolve().parents[1]
if str(H_DIR) not in sys.path:
    sys.path.insert(0, str(H_DIR))

from ui_fonts import choose_cjk_font_family  # noqa: E402


class UIFontTests(unittest.TestCase):
    def test_prefers_simplified_chinese_noto_font(self) -> None:
        self.assertEqual(
            choose_cjk_font_family(
                ["Droid Sans Fallback", "Noto Sans CJK SC"]
            ),
            "Noto Sans CJK SC",
        )

    def test_uses_installed_fallback_in_priority_order(self) -> None:
        self.assertEqual(
            choose_cjk_font_family(["AR PL UKai CN", "Droid Sans Fallback"]),
            "Droid Sans Fallback",
        )

    def test_missing_cjk_font_does_not_invent_family(self) -> None:
        self.assertEqual(choose_cjk_font_family(["DejaVu Sans"]), "")

    def test_jetson_tk_x11_chinese_alias_is_supported(self) -> None:
        self.assertEqual(
            choose_cjk_font_family(["gothic", "song ti", "fixed"]),
            "song ti",
        )


if __name__ == "__main__":
    unittest.main()

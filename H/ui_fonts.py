#!/usr/bin/env python3
"""Tk界面中文字体检测与统一配置。"""

from typing import Iterable


CJK_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "Droid Sans Fallback",
    "AR PL UKai CN",
    "AR PL UMing CN",
    "WenQuanYi Micro Hei",
    # Jetson上的旧X11/Tk有时只能看到这些字体别名，Fontconfig中的
    # Noto/Droid真实家族名不会出现在tkfont.families()里。
    "song ti",
    "fangsong ti",
)


def choose_cjk_font_family(available_families: Iterable[str]) -> str:
    available = {str(name).strip() for name in available_families}
    for candidate in CJK_FONT_CANDIDATES:
        if candidate in available:
            return candidate
    # Tk一定有TkDefaultFont；返回空字符串会让调用处沿用其实际字体，
    # 比虚构一个不存在的字体名称更可靠。
    return ""


def configure_tk_cjk_fonts(root, tkfont, ttk, size: int = 12):
    """选择真实存在的中文字体并覆盖Tk和ttk全部常用控件。"""

    family = choose_cjk_font_family(tkfont.families(root))
    font_options = {"size": int(size)}
    if family:
        font_options["family"] = family
    for font_name in (
        "TkDefaultFont",
        "TkTextFont",
        "TkFixedFont",
        "TkMenuFont",
        "TkHeadingFont",
        "TkCaptionFont",
        "TkSmallCaptionFont",
        "TkIconFont",
        "TkTooltipFont",
    ):
        try:
            tkfont.nametofont(font_name).configure(**font_options)
        except Exception:
            # 不同Tk版本提供的命名字体集合并不完全一致。
            continue
    try:
        tkfont.nametofont("TkHeadingFont").configure(
            size=int(size) + 1, weight="bold"
        )
    except Exception:
        pass

    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass
    effective_family = family or tkfont.nametofont("TkDefaultFont").actual(
        "family"
    )
    # “.”是ttk根样式，可覆盖Combobox、Spinbox和Notebook.Tab等主题中
    # 容易继续使用西文字体的控件。
    style.configure(".", font=(effective_family, int(size)))
    for style_name in (
        "TLabel",
        "TButton",
        "TCheckbutton",
        "TRadiobutton",
        "TEntry",
        "TSpinbox",
        "TCombobox",
        "TLabelframe.Label",
        "TNotebook.Tab",
    ):
        style.configure(style_name, font=(effective_family, int(size)))
    return effective_family, style

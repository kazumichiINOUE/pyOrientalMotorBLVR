'''
[Tokyo Night Light](https://github.com/tokyo-night/tokyo-night-vscode-theme?tab=readme-ov-file#tokyo-night-light)
#8c4351 This keyword, HTML elements, Regex group symbol, CSS units, Terminal Red
#965027 Number and Boolean constants, Language support constants
#8f5e15 Function parameters, Regex character sets, Terminal Yellow
#634f30 Parameters inside functions (semantic highlighting only)
#385f0d Strings, CSS class names
#33635c Object literal keys, Markdown links, Regex literal strings, Terminal Green
#006c86 Language support functions, CSS HTML elements
#0f4b6e Object properties, Regex quantifiers and flags, Terminal Cyan, Markdown code, Import/export keywords
#2959aa Function names, CSS property names, Markdown Headings, Terminal Blue
#5a3e8e Control Keywords, Storage Types, Regex symbols and operators, HTML Attributes, Terminal Magenta
#343b58 Editor Foreground, Variables, Class names, Terminal White
#40434f Markdown Text, HTML Text
#343B58 Terminal Black
#6c6e75 Comments
#e6e7ed Editor Background
'''

# 色指定の形式変換
def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))


import lupa

def read_config(fname = 'config.lua'):
    # Luaインタープリタを初期化
    lua = lupa.LuaRuntime()
    # config.luaファイルを読み込む
    with open(fname, 'r', encoding='utf-8') as f:
        _config = lua.execute(f.read())
    
    return _config
    
def read_mapInfo(fname = 'mapInfo.lua'):
    # Luaインタープリタを初期化
    lua = lupa.LuaRuntime()
    # mapInfo.luaファイルを読み込む
    with open(fname, 'r', encoding='utf-8') as f:
        _config = lua.execute(f.read())
    
    return _config

def read_wp(fname = 'wp_list.lua'):
    # Luaインタープリタを初期化
    lua = lupa.LuaRuntime(unpack_returned_tuples=True)
    # Luaスクリプトを読み込む
    with open(fname, 'r', encoding='utf-8') as f:
        lua.execute(f.read())
    # Luaのグローバル変数 `wp_list` を取得
    wp_list = lua.globals().wp_list
    print(wp_list)
    return wp_list

    # Pythonのリストとして返す
    return [list(wp) for wp in wp_list]
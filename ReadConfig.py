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
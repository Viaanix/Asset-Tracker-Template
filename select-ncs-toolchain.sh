declare -A toolchain_map

toolchain_map["v3.1.0-linux"]="c5be9c56c7"
toolchain_map["v3.1.0-windows"]="b8b84efebd"

if [[ "$OSTYPE" == "linux"* ]]; then
    VX_NCS_TOOLCHAIN=$HOME/ncs/toolchains/${toolchain_map["v3.1.0-linux"]}
else # mysys for Window
    VX_NCS_TOOLCHAIN=/c/ncs/toolchains/${toolchain_map["v3.1.0-windows"]}
fi
# This script should be sourced, not executed.

source ./select-ncs-toolchain.sh

GREEN="\033[0;32m"
YELLOW="\033[0;33m"
RED="\033[0;31m"
NC="\033[0m"

# As this script is not using shebang instead of use $0 lets use BASH_SOURCE
WEST_WORKSPACE=$(dirname "$(dirname "$(realpath "$(dirname "${BASH_SOURCE[0]}")")")")
echo "West workspace to" $WEST_WORKSPACE

create_python_symlink() {
    local vx_ncs_toolchain_binaries="$1"
    local python_link="$vx_ncs_toolchain_binaries/python"
    local python3="$vx_ncs_toolchain_binaries/python3"

    if [ ! -f "$python_link" ]; then
        if [ -f "$python3" ]; then
            ln -s "$python3" "$python_link"
            echo "Created symbolic link: $python_link -> $python3"
        else
            echo "Error: $python3 does not exist. Cannot create symbolic link."
        fi
    else
        echo "Symbolic link $python_link already exists."
    fi
}


# Add path folder to PATH if it's not already present
add_to_path() {
    local new_path=$1
    if [[ ":$PATH:" != *":$new_path:"* ]]; then
        export PATH="$new_path:$PATH"
        echo -e "${GREEN}Added $new_path to PATH${NC}"
    else
        echo -e "${YELLOW}$new_path is already in PATH${NC}"
    fi
}

if [[ "$OSTYPE" == "linux"* ]]; then
    echo "Os: Linux / GNU"
    VX_NCS_TOOLCHAIN_BINARIES=$VX_NCS_TOOLCHAIN/usr/local/bin
    add_to_path "$VX_NCS_TOOLCHAIN_BINARIES"
    add_to_path "$VX_NCS_TOOLCHAIN/usr/local/cmake"
    export LD_LIBRARY_PATH=$VX_NCS_TOOLCHAIN/usr/local/lib:$LD_LIBRARY_PATH
    create_python_symlink "$VX_NCS_TOOLCHAIN_BINARIES"
    PYTHON_BIN=$(which python)
else # mysys for Window
    echo "Os: Window"
    add_to_path "$VX_NCS_TOOLCHAIN/mingw64/bin"
    add_to_path "$VX_NCS_TOOLCHAIN/opt/bin"
    add_to_path "$VX_NCS_TOOLCHAIN/opt/bin/Scripts"
    PYTHON_BIN=$(which python)
fi

echo "CMake binary located at: $(which cmake)"
echo "Using Python interpreter at: $PYTHON_BIN"
echo "Using west interpreter at: $(which west)"

export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=$VX_NCS_TOOLCHAIN/opt/zephyr-sdk
add_to_path "$ZEPHYR_SDK_INSTALL_DIR"

ZEPHYR_ENV_SCRIPT=$WEST_WORKSPACE/zephyr/zephyr-env.sh

if [[ -f "$ZEPHYR_ENV_SCRIPT" ]]; then
    source "$ZEPHYR_ENV_SCRIPT"
    west zephyr-export
else
    echo -e "${RED}Error: '$ZEPHYR_ENV_SCRIPT' does not exist."
    echo -e "This is probably because you are setting up env vars but west update has not been yet issued.${NC}"
    return 1
fi
#!/usr/bin/env python3
"""
Simple generator to populate .vscode JSON templates with platform-specific values
based on .vscode/platform.json. It replaces placeholders in the existing
.vscode JSON files and writes updated files back.

Placeholders supported:
- ${TOOLCHAIN_GCC}
- ${GDB_PATH}
- ${OPENOCD_PATH}
- ${BUILD_DIR}
- ${COMPILE_COMMANDS}
- ${STM32_DEVICE}
- ${STM32_TARGET}
- ${ELF_NAME}
- ${SVD_FILE}

Run from the workspace root: python3 generate_vscode_files.py
"""
import json
import os
import platform
import re
import sys

ROOT = os.path.abspath(os.path.dirname(__file__))
PLATFORM_FILE = os.path.join(ROOT, 'platform.json')
TEMPLATES = ['c_cpp_properties.json', 'launch.json', 'tasks.json', 'gcc-arm-none-eabi.cmake']

# Embedded templates — the generator will write these files with platform values
EMBED_TEMPLATES = {
    'c_cpp_properties.json': '''{
    "configurations": [
        {
            "name": "Auto",
            "includePath": [
                "${workspaceFolder}/**"
            ],
            "defines": [
                "${STM32_DEVICE}",
                "USE_HAL_DRIVER"
            ],
            "compilerPath": "${TOOLCHAIN_GCC}",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "gcc-arm",
            "compileCommands": "${COMPILE_COMMANDS}",
            "configurationProvider": "ms-vscode.cmake-tools"
        }
    ],
    "version": 4
}
''',

    'launch.json': '''{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (OpenOCD)",
            "type": "cortex-debug",
            "request": "launch",
            "executable": "${workspaceFolder}/${BUILD_DIR}/${ELF_NAME}.elf",

            "servertype": "openocd",
            "gdbPath": "${GDB_PATH}",

            "configFiles": [
                "interface/stlink.cfg",
                "target/${STM32_TARGET}"
            ],

            "runToEntryPoint": "main",
            "svdFile": "${SVD_FILE}",
            "preLaunchTask": "CMake: Build (Debug)"
        },
        {
            "name": "Attach (OpenOCD)",
            "type": "cortex-debug",
            "request": "attach",
            "executable": "${workspaceFolder}/${BUILD_DIR}/${ELF_NAME}.elf",

            "servertype": "openocd",
            "gdbPath": "${GDB_PATH}",

            "configFiles": [
                "interface/stlink.cfg",
                "target/${STM32_TARGET}"
            ],

            "svdFile": "${SVD_FILE}",
            "preLaunchTask": "CMake: Build (Debug)"
        }
    ]
}
''',

    'tasks.json': '''{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "CMake: Configure (Debug)",
            "type": "shell",
            "command": "cmake",
            "args": [
                "--preset",
                "Debug"
            ],
            "group": "build",
            "problemMatcher": []
        },
        {
            "label": "CMake: Build (Debug)",
            "type": "shell",
            "command": "cmake",
            "args": [
                "--build",
                "${workspaceFolder}/${BUILD_DIR}",
                "--config",
                "Debug",
                "--target",
                "all",
                "-j",
                "10",
                "--verbose"
            ],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": [
                "$gcc"
            ],
            "dependsOn": [
                "CMake: Configure (Debug)"
            ]
        },
        {
            "label": "CMake: Clean",
            "type": "shell",
            "command": "cmake",
            "args": [
                "--build",
                "${workspaceFolder}/${BUILD_DIR}",
                "--target",
                "clean"
            ],
            "group": "build",
            "problemMatcher": []
        },
        {
            "label": "CMake: Build (Release)",
            "type": "shell",
            "command": "cmake",
            "args": [
                "--build",
                "${workspaceFolder}/build/Release",
                "--config",
                "Release",
                "--target",
                "all",
                "-j",
                "10"
            ],
            "group": "build",
            "problemMatcher": [
                "$gcc"
            ]
        },
        {
            "label": "Flash Firmware",
            "type": "shell",
            "command": "${OPENOCD_PATH}",
            "args": [
                "-f",
                "interface/stlink.cfg",
                "-f",
                "target/${STM32_TARGET}",
                "-c",
                "program ${BUILD_DIR}/${ELF_NAME}.hex reset exit"
            ],
            "group": "build",
            "problemMatcher": [],
            "dependsOn": [
                "CMake: Build (Debug)"
            ],
            "options": {
                "cwd": "${workspaceFolder}"
            }
        }
    ]
}
''',

    'gcc-arm-none-eabi.cmake': '''set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_ROOT ${TOOLCHAIN_BIN_DIR})
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_ROOT}/${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_ROOT}/${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_ROOT}/${TOOLCHAIN_PREFIX}g++)
set(CMAKE_AR                        ${TOOLCHAIN_ROOT}/${TOOLCHAIN_PREFIX}ar)
set(CMAKE_RANLIB                    ${TOOLCHAIN_ROOT}/${TOOLCHAIN_PREFIX}ranlib)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_ROOT}/${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_ROOT}/${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \\"${CMAKE_SOURCE_DIR}/STM32H753XX_FLASH.ld\\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
'''
}

def load_platform_config():
    with open(PLATFORM_FILE, 'r', encoding='utf-8') as f:
        cfg = json.load(f)
    sysname = platform.system().lower()
    if 'linux' in sysname:
        key = 'linux'
    elif 'windows' in sysname:
        key = 'windows'
    else:
        key = 'linux'
    return cfg.get(key, cfg.get('linux'))

def detect_elf_name():
    """Auto-detect the ELF name from CMakeLists.txt by parsing CMAKE_PROJECT_NAME."""
    cmake_file = os.path.join(ROOT, 'CMakeLists.txt')
    if not os.path.exists(cmake_file):
        return None
    
    with open(cmake_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Look for set(CMAKE_PROJECT_NAME xxx)
    match = re.search(r'set\s*\(\s*CMAKE_PROJECT_NAME\s+([^\s\)]+)\s*\)', content)
    if match:
        return match.group(1)
    
    # Also try to find project(xxx)
    match = re.search(r'project\s*\(\s*([^\s\)]+)\s*\)', content)
    if match:
        project_name = match.group(1)
        # Check if it's a variable reference
        if project_name.startswith('${') and project_name.endswith('}'):
            return None  # Already handled by CMAKE_PROJECT_NAME
        return project_name
    
    return None

def replace_placeholders(text, mapping):
    for k, v in mapping.items():
        text = text.replace('${' + k + '}', v)
    return text

def main():
    cfg = load_platform_config()
    
    # Auto-detect ELF name from CMakeLists.txt if not specified
    elf_name = cfg.get('elf_name')
    if not elf_name:
        detected_elf = detect_elf_name()
        if detected_elf:
            elf_name = detected_elf
            print(f'Auto-detected ELF name from CMakeLists.txt: {elf_name}')
        else:
            elf_name = 'firmware'  # Fallback default
            print(f'Warning: Could not detect ELF name, using default: {elf_name}')
    
    # Extract toolchain bin directory from toolchain_gcc path
    toolchain_gcc = cfg.get('toolchain_gcc', '')
    toolchain_bin_dir = os.path.dirname(toolchain_gcc) if toolchain_gcc else ''
    
    mapping = {
        'TOOLCHAIN_GCC': toolchain_gcc,
        'TOOLCHAIN_BIN_DIR': toolchain_bin_dir,
        'GDB_PATH': cfg.get('gdb', ''),
        'OPENOCD_PATH': cfg.get('openocd', ''),
        'BUILD_DIR': cfg.get('build_dir', 'build/Debug'),
        'COMPILE_COMMANDS': cfg.get('compile_commands', '${workspaceFolder}/build/Debug/compile_commands.json'),
        'STM32_DEVICE': cfg.get('stm32_device', 'STM32L476xx'),
        'STM32_TARGET': cfg.get('stm32_target', 'stm32l4x.cfg'),
        'ELF_NAME': elf_name,
        'SVD_FILE': cfg.get('svd_file', '')
    }

    vscode_dir = os.path.join(ROOT, '.vscode')
    cmake_dir = os.path.join(ROOT, 'cmake')

    # Ensure directories exist
    os.makedirs(vscode_dir, exist_ok=True)
    os.makedirs(cmake_dir, exist_ok=True)

    for name in TEMPLATES:
        template = EMBED_TEMPLATES.get(name)
        if template is None:
            print('No embedded template for', name)
            continue
        new_text = replace_placeholders(template, mapping)
        
        # Write cmake files to cmake/ directory, others to .vscode/
        if name.endswith('.cmake'):
            path = os.path.join(cmake_dir, name)
        else:
            path = os.path.join(vscode_dir, name)
        
        with open(path, 'w', encoding='utf-8') as f:
            f.write(new_text)
        print('Wrote', path)

    print('Generation complete. Reload window in VS Code if necessary.')

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

from os import system, environ

def tmux(command):
    system(f'tmux {command}')

def tmux_shell(command, run=True):
    if run:
        tmux('send-keys "%s" "C-m"' % command)
    else:
        tmux(f'send-keys "{command}"')        

def new_window(name, command, run=True):
    # print("new_window:", name, command)
    tmux("new-window")
    tmux(f'rename-window {name}')
    tmux_shell(command, run=run)

def ros2(name, command, run=True):
    new_window(name, f'ros2run', run=True)
    tmux_shell(command, run=run)

def turtle4(name, command, run=True):
    new_window(name, f'turtle4', run=True)
    tmux_shell(command, run=run)
    
def husky(name, command, run=True):
    new_window(name, f'husky', run=True)
    tmux_shell(command, run=run)
    
def gpu2(name, command, run=True):
    new_window(name, f'ros2run', run=True)
    tmux_shell("gpusetup", run=run)
    tmux_shell(command, run=run)

import asyncio
import os
import subprocess

cmd_gui_server = 'xbot2_gui_server'
cmd_gui_client = '~/.xbot2_gui_client/xbot2_gui_client_x86_64/bin/xbot2_gui'

# check if xbot2_gui_client exists, if not download it
if not os.path.exists( os.path.expanduser(cmd_gui_client) ):

    print('Downloading xbot2_gui_client...')
    subprocess.run(f'wget -P /tmp https://github.com/ADVRHumanoids/robot_monitoring/releases/latest/download/xbot2_gui_client_x86_64.zip',
                   shell=True,
                   timeout=120.0,
                   check=True)
    
    print('Extracting xbot2_gui_client... to ~/.xbot2_gui_client')
    subprocess.run(f'rm -rf ~/.xbot2_gui_client && mkdir ~/.xbot2_gui_client && unzip -o /tmp/xbot2_gui_client_x86_64.zip -d ~/.xbot2_gui_client',
                   shell=True,
                   timeout=120.0,
                   check=True)


# run command and print output in real time
async def run_command(cmd, name):
    proc = await asyncio.create_subprocess_shell(
        cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT
    )

    while True:
        line = await proc.stdout.readline()
        if not line:
            break
        print(f'[{name}] {line.decode()}', end='', flush=True)

    retcode = await proc.wait()

    print(f'[{name}] process exited with {retcode}')


def main():
    # spawn tasks
    asyncio.get_event_loop().create_task(run_command(cmd_gui_server, 'gui_srv'))
    asyncio.get_event_loop().create_task(run_command(cmd_gui_client, 'gui_cli'))

    # then, loop forever
    asyncio.get_event_loop().run_forever()

import asyncio

class Process:

    def __init__(self, name, cmd, machine, loop=None):
        self.name = name
        self.proc: asyncio.subprocess.Process = None
        self.cmd = cmd
        self.hostname = machine
        self.cmdline = dict()
        

    async def attach(self):

        """
        Try to attach to an existing screen session, without creating
        one.

        Returns:
            True if the session exists
        """

        # if proc is not none, we have an alive session
        if self.proc is not None:
            return True
        
        # first check if session running, attach if it is
        cmd = f'screen -xr {self.name}'
        args = ['-tt', self.hostname, cmd]
        tmp_proc = await asyncio.create_subprocess_exec('/usr/bin/ssh', *args,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)

        # if this command fails immediately, no session exists 
        try:
            # give it 1 sec
            retcode = await asyncio.wait_for(tmp_proc.wait(), 1.0)
            return False

        except asyncio.TimeoutError:
            # timeout expired, we got the session
            print('got session')

            # run lifetime handler
            self.proc = tmp_proc
            asyncio.get_running_loop().create_task(self._lifetime_handler())

            return True

    async def start(self):

        """
        Start a new screen session. Attach to an existing one if possible.

        Returns:
            True
        """
        
        # first check if session running, attach if it is
        await self.attach()

        if self.proc is not None:
            return True

        # create new session
        cmd = f'rm -rf /tmp/{self.name}_log; bash -ic "screen -mS {self.name} -L -Logfile /tmp/{self.name}_log {self.cmd}"'
        args = ['-tt', self.hostname, cmd]
        print('executing command ssh ' + ' '.join(args))
        self.proc = await asyncio.create_subprocess_exec('/usr/bin/ssh', *args,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)
        print('DONE executing command ssh ' + ' '.join(args))

        # run lifetime handler
        asyncio.get_running_loop().create_task(self._lifetime_handler())

        return True
    
    async def stop(self):
        """
        Send CTRL+C to the remote session
        """
        self.proc.stdin.write('\x03\n'.encode())
        await self.proc.stdin.drain()  

    async def status(self):
        """
        Check if session is running by trying to attach
        to it. Not super cheap.
        """

        await self.attach()

        if self.proc is None:
            return 'Stopped'
        else:
            return 'Running'

        

    async def _lifetime_handler(self):
        """
        Task to set self.proc to None after session exit
        """
        
        if self.proc is None:
            return

        print(f'started task _lifetime_handler for {self.name}')

        retcode = await self.proc.wait()

        print(f'process {self.name} died with exit code {retcode}')

        self.proc = None

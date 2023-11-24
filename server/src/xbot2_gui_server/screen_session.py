import asyncio
import builtins

class Process:

    def __init__(self, name, cmd, machine, loop=None):
        self.name = name
        self.stdout_proc: asyncio.subprocess.Process = None
        self.stderr_proc: asyncio.subprocess.Process = None
        self.ssh_session: asyncio.subprocess.Process = None
        self.running = False
        self.cmd = cmd
        self.hostname = machine
        self.cmdline = dict()
        asyncio.get_event_loop().create_task(self._keep_ssh_session_running())

    
    async def execute_command(machine, cmd):
        
        proc  = await asyncio.create_subprocess_exec('/usr/bin/ssh',
                    # '-tt', 
                    machine,
                    f'bash -ic "{cmd}"',
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE)
        
        stdin = proc._noop()
        
        if proc.stdout is not None:
            stdout = proc._read_stream(1)
        else:
            stdout = proc._noop()
        
        if proc.stderr is not None:
            stderr = proc._read_stream(2)
        else:
            stderr = proc._noop()
        
        stdin, stdout, stderr = await asyncio.tasks.gather(stdin, stdout, stderr)
        
        ret = await proc.wait()
        
        return ret, stdout, stderr
        
        

    async def attach(self):

        """
        Try to attach to an existing screen session, without creating
        one.

        Returns:
            True if the session exists
        """

        # first check if session running, attach if it is
        cmd = f'tmux has-session -t {self.name}'
        args = ['-tt', self.hostname, cmd]
        proc = await asyncio.create_subprocess_exec('/usr/bin/ssh', *args,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)
        
        # give it 1 sec
        retcode = await asyncio.wait_for(proc.wait(), 1.0)
        if retcode == 0:
            print(f'{self.name} session detected on {self.hostname}')
            asyncio.get_running_loop().create_task(self._keep_ssh_session_running())
            await self._create_stdout_stderr_session()
            return True
        else: 
            print(f'{self.name} no session detected on {self.hostname}')
            return False
        
        # on timeout exception is raised


    async def start(self, options=None):

        """
        Start a new screen session. Attach to an existing one if possible.

        Returns:
            True
        """
        
        # first check if session running, attach if it is
        session_exists = await self.attach()

        if session_exists:
            return True

        # parse options
        opt_cmd = ''
        if options is not None:
            opt_cmd = self._parse_options(options)

        # create new session
        cmd = f'bash -ic "tmux new-session -s {self.name} \
-d \
\\"{self.cmd} {opt_cmd} \
1> >(tee /tmp/{self.name}.stdout) \
2> >(tee /tmp/{self.name}.stderr >&2); sleep 1\\""'
        
        args = ['-tt', self.hostname, cmd]
        print('executing command ssh ' + ' '.join(args))
        proc = await asyncio.create_subprocess_exec('/usr/bin/ssh', *args,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)

        # give it 1 sec
        retcode = await asyncio.wait_for(proc.wait(), 1.0)
        if retcode == 0:
            print(f'{self.name} session started on {self.hostname}')
            asyncio.get_running_loop().create_task(self._keep_ssh_session_running())
            await self._create_stdout_stderr_session()
            return True
        else: 
            print(f'{self.name} failed to start session on {self.hostname}')
            return False

    
    async def stop(self):
        """
        Send CTRL+C to the remote session
        """
        
        # send ctrl c
        cmd = f'tmux send-keys -t {self.name} C-c'
        args = ['-tt', self.hostname, cmd]
        proc = await asyncio.create_subprocess_exec('/usr/bin/ssh', *args,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)
        
        # give it 1 sec
        retcode = await asyncio.wait_for(proc.wait(), 1.0)
        if retcode == 0:
            print(f'[{self.name}] sent ctrl+c to target session')
            return True
        else: 
            print(f'[{self.name}] could NOT send ctrl+c to target session')
            return False
        
    
    async def kill(self):
        """
        Send CTRL+C to the remote session
        """
        
        # send ctrl c
        cmd = f'tmux send-keys -t {self.name} C-\\'
        args = ['-tt', self.hostname, cmd]
        proc = await asyncio.create_subprocess_exec('/usr/bin/ssh', *args,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)
        
        # give it 1 sec
        retcode = await asyncio.wait_for(proc.wait(), 1.0)
        if retcode == 0:
            print(f'[{self.name}] sent ctrl+\\ to target session')
            return True
        else: 
            print(f'[{self.name}] could NOT send ctrl+\\ to target session')
            return False


    async def status(self):
        """
        Check if session is running by trying to attach
        to it. Not super cheap.
        """

        self.running = await self._screen_session_running(self.name)

        if self.running:
            return 'Running'
        else:
            return 'Stopped'
        
    
    async def read_stdout(self):
        while True:
            yield await self.stdout_proc.stdout.readline()


    async def read_stderr(self):
        while True:
            yield await self.stderr_proc.stdout.readline()
        

    async def _create_ssh_session(self):
        
        ssh_proc = await asyncio.create_subprocess_exec(
                '/usr/bin/ssh', '-o ConnectTimeout=3', self.hostname,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)
        
        while True:
            try:
                line = await asyncio.wait_for(ssh_proc.stdout.readline(), 1.0)
            except asyncio.TimeoutError:
                break
        
        print(f'[{self.name}] ssh session ready')
        return ssh_proc
    

    async def _create_stdout_stderr_session(self):
        
        if self.stderr_proc:
            self.stderr_proc.kill()

        if self.stdout_proc:
            self.stdout_proc.kill()
        
        cmd = f'touch /tmp/{self.name}.stdout && tail -f -n +1 /tmp/{self.name}.stdout'
        
        args = ['-tt', self.hostname, cmd]

        self.stdout_proc =  await asyncio.create_subprocess_exec(
                                        '/usr/bin/ssh', *args,
                                        stdout=asyncio.subprocess.PIPE,
                                        stderr=asyncio.subprocess.PIPE,
                                        stdin=asyncio.subprocess.PIPE)
        
        cmd = f'touch /tmp/{self.name}.stderr && tail -f -n +1 /tmp/{self.name}.stderr'
        
        args = ['-tt', self.hostname, cmd]

        self.stderr_proc =  await asyncio.create_subprocess_exec(
                                        '/usr/bin/ssh', *args,
                                        stdout=asyncio.subprocess.PIPE,
                                        stderr=asyncio.subprocess.PIPE,
                                        stdin=asyncio.subprocess.PIPE)
        
        return self.stdout_proc, self.stderr_proc
        

    async def _keep_ssh_session_running(self):

        while True:
            self.ssh_session = await self._create_ssh_session()
            retcode = await self.ssh_session.wait()
            self.ssh_session = None
            print(f'ssh session with {self.hostname} died with exit code {retcode}, restarting..')

    
    async def _screen_session_running(self, name):

        lock = asyncio.Lock()
        
        async with lock:

            def print(string):
                pass #builtins.print(string)

            # make sure we have our ssh sesssion
            if self.ssh_session is None:
                builtins.print(f'{self.name} ssh session offline')
                return False

            # consume stdout
            read_coro =  self.ssh_session.stdout.read(4096)
            try:
                await asyncio.wait_for(read_coro, timeout=0.1)
            except:
                pass
            
            # send command
            print(f'issue tmux has-session -t {self.name} over ssh..')
            self.ssh_session.stdin.write(f'tmux has-session -t {self.name} 1>/dev/null 2>/dev/null;\necho $?\n'.encode())
            await self.ssh_session.stdin.drain()
            print('..done')
            
            line = (await self.ssh_session.stdout.readline()).decode().strip()
            print(f'got line "{line}"')
            
            return line == '0'


    def _parse_options(self, options):

        cmdline = ''
        
        for k, v in options.items():

            opt_type = self.cmdline[k]['type']

            if opt_type == 'check':
                if v:
                    cmdline += self.cmdline[k]['arg']
            elif opt_type == 'combo':
                options = self.cmdline[k]['options']
                if v in options:
                    index = options.index(v)
                    cmdline += self.cmdline[k]['arg'][index]
            elif opt_type == 'text':
                if v:
                     cmdline += self.cmdline[k]['arg'].format(__value__=v)
            else:
                raise KeyError(f'invalid option type {opt_type}')
            
            cmdline += ' '
        
        return cmdline

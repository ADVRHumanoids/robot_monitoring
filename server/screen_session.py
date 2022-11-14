import asyncio
import parse

class Process:

    def __init__(self, name, cmd, machine, loop=None):
        self.name = name
        self.proc: asyncio.subprocess.Process = None
        self.cmd = cmd
        self.hostname = machine
        self.cmdline = dict()
        self.ssh_session = None
        asyncio.get_event_loop().create_task(self._keep_ssh_session_running())
        

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

    async def start(self, options=None):

        """
        Start a new screen session. Attach to an existing one if possible.

        Returns:
            True
        """
        
        # first check if session running, attach if it is
        await self.attach()

        if self.proc is not None:
            return True

        # parse options
        opt_cmd = ''
        if options is not None:
            opt_cmd = self._parse_options(options)
            print(opt_cmd)

        # create new session
        cmd = f'rm -rf /tmp/{self.name}_log; bash -ic "screen -mS {self.name} -L -Logfile /tmp/{self.name}_log {self.cmd} {opt_cmd}"'
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
        if self.proc is None:
            await self.attach()
        self.proc.stdin.write('\x03\n'.encode())
        await self.proc.stdin.drain()  

    async def status(self):
        """
        Check if session is running by trying to attach
        to it. Not super cheap.
        """

        is_running = await self._screen_session_running(self.name)

        if is_running:
            return 'Running'
        else:
            return 'Stopped'

    async def _create_ssh_session(self):
        return await asyncio.create_subprocess_exec(
                '/usr/bin/ssh', '-o ConnectTimeout=3', self.hostname,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)

    async def _keep_ssh_session_running(self):

        while True:
            self.ssh_session = await self._create_ssh_session()
            retcode = await self.ssh_session.wait()
            print(f'ssh session died with exit code {retcode}, restarting..')

    
    async def _screen_session_running(self, name):

        # def print(string):
        #     pass

        # make sure we have our ssh sesssion
        if self.ssh_session is None:
            self.ssh_session = self._create_ssh_session()

        # consume stdout
        read_coro =  self.ssh_session.stdout.read(4096)
        try:
            await asyncio.wait_for(read_coro, timeout=0.1)
        except:
            pass
        
        # send command
        print('issue screen -ls over ssh..')
        self.ssh_session.stdin.write('screen -ls \n'.encode())
        await self.ssh_session.stdin.drain()
        print('..done')
        
        res = None
        while True:
            print(f'[{name}] reading line..')
            line = await self.ssh_session.stdout.readline()
            line = line.decode().strip()
            print(f'got line "{line}"')
            
            if line.startswith('No Sockets found'):
                print('not running (no sockets found)')
                res = False
                break
            elif len(line) == 0:
                print('empty')
                continue
            else:
                end_line =  parse.parse('{} Socket{}in {}', line)
                print(f'end_line? {end_line}')
                if end_line is not None:
                    print(f'not running (but {end_line[0]} sockets were found)')
                    res = False
                    break
                
                session_line = parse.parse('{}.{}\t({})\t({})', line)
                print(f'[{name}] session_line? {session_line}')
                if session_line is not None and session_line[1] == name:
                    print(f'{name} found!')
                    res = True
                    break
        
        return res

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

    def _parse_options(self, options):

        cmdline = ''
        
        for k, v in options.items():

            opt_type = self.cmdline[k]['type']

            if opt_type == 'check':
                if v:
                    cmdline += self.cmdline[k]['cmd']
            elif opt_type == 'combo':
                options = self.cmdline[k]['options']
                if v in options:
                    index = options.index(v)
                    cmdline += self.cmdline[k]['cmd'][index]
            elif opt_type == 'text':
                if v:
                     cmdline += self.cmdline[k]['cmd'].format(__value__=v)
            else:
                raise KeyError(f'invalid option type {opt_type}')
            
            cmdline += ' '
        
        return cmdline

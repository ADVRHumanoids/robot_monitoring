import { Root, Type } from 'protobufjs'
import type {
  ConnectionState,
  JsonStreamMessage,
  JointTelemetry,
  StreamEvent,
} from '../types'
import { normalizeTelemetry } from '../model'

export interface MonitorTransportCallbacks {
  onConnectionChange: (state: ConnectionState) => void
  onEvent: (event: StreamEvent) => void
  onProtocolError?: (error: Error) => void
}

type DecodedEnvelope = {
  text?: { text?: string }
  jointstate?: Partial<JointTelemetry>
  processOutput?: { name?: string; out?: string; err?: string }
}

export function decodeJsonFrame(text: string): StreamEvent {
  return { kind: 'json', value: JSON.parse(text) as JsonStreamMessage }
}

/** Pure decoding boundary, exported so wire compatibility can be fixture-tested. */
export function decodeEnvelope(messageType: Type, bytes: Uint8Array): StreamEvent[] {
  const decoded = messageType.decode(bytes)
  const envelope = messageType.toObject(decoded, {
    arrays: true,
    objects: true,
    defaults: false,
  }) as DecodedEnvelope
  const events: StreamEvent[] = []

  if (envelope.text?.text) events.push(decodeJsonFrame(envelope.text.text))
  if (envelope.jointstate) {
    events.push({ kind: 'joint-state', value: normalizeTelemetry(envelope.jointstate) })
  }
  if (envelope.processOutput) {
    events.push({
      kind: 'process-output',
      name: envelope.processOutput.name ?? 'unknown',
      out: envelope.processOutput.out ?? '',
      err: envelope.processOutput.err ?? '',
    })
  }
  return events
}

/**
 * Owns the complete streaming lifecycle.
 *
 * The server sometimes sends a plain WebSocket text frame (heartbeats) and
 * sometimes a binary protobuf Message. A protobuf Message can in turn contain
 * JSON text, joint telemetry, or process output. Keeping this nesting here
 * means Vue components only see application-level events.
 */
export class MonitorTransport {
  private socket?: WebSocket
  private messageType?: Type
  private reconnectTimer?: number
  private reconnectAttempt = 0
  private shouldRun = false

  constructor(private readonly callbacks: MonitorTransportCallbacks) {}

  async start(): Promise<void> {
    this.shouldRun = true
    this.callbacks.onConnectionChange('connecting')
    try {
      await this.loadProtocol()
      this.connect()
    } catch (reason) {
      this.reportError(reason)
      this.scheduleReconnect()
    }
  }

  stop(): void {
    this.shouldRun = false
    if (this.reconnectTimer !== undefined) window.clearTimeout(this.reconnectTimer)
    this.socket?.close()
    this.callbacks.onConnectionChange('disconnected')
  }

  private async loadProtocol(): Promise<void> {
    if (this.messageType) return

    // Vite copies the repository's proto/ directory beside index.html. Loading
    // generic.proto lets protobufjs resolve its imports from that same folder.
    const root = new Root()
    await root.load(`${import.meta.env.BASE_URL}generic.proto`)
    root.resolveAll()
    this.messageType = root.lookupType('Message')
  }

  private connect(): void {
    if (!this.shouldRun || !this.messageType) return

    const scheme = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    this.socket = new WebSocket(`${scheme}//${window.location.host}/ws`)
    this.socket.binaryType = 'arraybuffer'

    this.socket.onopen = () => {
      this.reconnectAttempt = 0
      this.callbacks.onConnectionChange('connected')

      // This is a subscription/transport handshake, not a robot command. It
      // asks aiohttp to mirror the UDP telemetry stream into this WebSocket.
      this.socket?.send(JSON.stringify({ type: 'request_ws_udp_tunnel' }))
    }

    this.socket.onmessage = (event) => {
      void this.handleFrame(event.data)
    }

    this.socket.onerror = () => {
      // onclose performs the reconnect. Reporting here provides useful detail
      // without starting two concurrent reconnect timers.
      this.reportError(new Error('WebSocket transport error'))
    }

    this.socket.onclose = () => {
      if (this.shouldRun) this.scheduleReconnect()
    }
  }

  private scheduleReconnect(): void {
    if (!this.shouldRun || this.reconnectTimer !== undefined) return
    this.callbacks.onConnectionChange('reconnecting')

    const delay = Math.min(500 * 2 ** this.reconnectAttempt, 5_000)
    this.reconnectAttempt += 1
    this.reconnectTimer = window.setTimeout(async () => {
      this.reconnectTimer = undefined
      try {
        await this.loadProtocol()
        this.connect()
      } catch (reason) {
        this.reportError(reason)
        this.scheduleReconnect()
      }
    }, delay)
  }

  private async handleFrame(data: string | ArrayBuffer | Blob): Promise<void> {
    try {
      if (typeof data === 'string') {
        this.emitJson(data)
        return
      }

      const bytes = data instanceof Blob ? await data.arrayBuffer() : data
      decodeEnvelope(this.messageType!, new Uint8Array(bytes)).forEach((event) =>
        this.callbacks.onEvent(event),
      )
    } catch (reason) {
      // One corrupt or future message must not tear down the live connection.
      this.reportError(reason)
    }
  }

  private emitJson(text: string): void {
    this.callbacks.onEvent(decodeJsonFrame(text))
  }

  private reportError(reason: unknown): void {
    const error = reason instanceof Error ? reason : new Error(String(reason))
    this.callbacks.onProtocolError?.(error)
  }
}

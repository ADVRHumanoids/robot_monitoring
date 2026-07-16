import { resolve } from 'node:path'
import { Root } from 'protobufjs'
import { beforeAll, describe, expect, it } from 'vitest'
import { decodeEnvelope, decodeJsonFrame } from '../services/transport'
import type { Type } from 'protobufjs'

let messageType: Type

beforeAll(async () => {
  const root = new Root()
  await root.load(resolve(process.cwd(), '../proto/generic.proto'))
  root.resolveAll()
  messageType = root.lookupType('Message')
})

function encode(value: Record<string, unknown>): Uint8Array {
  return messageType.encode(messageType.create(value)).finish()
}

describe('XBot2 protobuf envelope', () => {
  it('decodes JSON text used by process and plugin status messages', () => {
    const events = decodeEnvelope(
      messageType,
      encode({ text: { text: JSON.stringify({ type: 'proc_status', name: 'xbot2', status: 'Running' }) } }),
    )
    expect(events[0]).toMatchObject({ kind: 'json', value: { type: 'proc_status', status: 'Running' } })
  })

  it('decodes process output and joint telemetry branches', () => {
    const output = decodeEnvelope(messageType, encode({ processOutput: { name: 'core', out: 'ready' } }))
    const joints = decodeEnvelope(messageType, encode({ jointstate: { linkPos: [1.25], vbatt: 48.2 } }))
    expect(output[0]).toMatchObject({ kind: 'process-output', name: 'core', out: 'ready' })
    expect(joints[0]).toMatchObject({ kind: 'joint-state', value: { linkPos: [1.25] } })
  })

  it('decodes plain heartbeat frames and rejects malformed JSON locally', () => {
    expect(decodeJsonFrame('{"type":"heartbeat"}')).toMatchObject({ value: { type: 'heartbeat' } })
    expect(() => decodeJsonFrame('not-json')).toThrow()
  })
})

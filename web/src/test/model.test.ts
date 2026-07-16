import { describe, expect, it } from 'vitest'
import {
  appendBoundedLog,
  buildJointRows,
  mergePluginStatistics,
  normalizeTelemetry,
  pluginUtilization,
} from '../model'
import type { JointInfo, ProcessLogEntry } from '../types'

const info: JointInfo = {
  success: true,
  message: 'ok',
  jnames: ['hip', 'knee'],
  qmin: [-1, -2],
  qmax: [1, 2],
  vmax: [3, 4],
  taumax: [50, 60],
  jstate: {},
}

describe('joint model', () => {
  it('aligns positional protobuf arrays with HTTP joint names and limits', () => {
    const telemetry = normalizeTelemetry({ linkPos: [0.2, 0.4], posRef: [0.3, 0.1], tor: [5, 6] })
    const rows = buildJointRows(info, telemetry, { knee: 'OVER_TEMP' })

    expect(rows[0]).toMatchObject({ name: 'hip', torque: 5, minimum: -1 })
    expect(rows[0]?.positionError).toBeCloseTo(0.1)
    expect(rows[1]).toMatchObject({ name: 'knee', fault: 'OVER_TEMP' })
    expect(rows[1]?.positionError).toBeCloseTo(-0.3)
  })

  it('uses undefined for missing or invalid array elements', () => {
    const rows = buildJointRows(info, normalizeTelemetry({ linkPos: [Number.NaN] }), {})
    expect(rows[0]?.linkPosition).toBeUndefined()
    expect(rows[1]?.linkPosition).toBeUndefined()
  })
})

describe('bounded process logs', () => {
  it('keeps the newest 200 entries', () => {
    let entries: ProcessLogEntry[] = []
    for (let id = 1; id <= 205; id += 1) {
      entries = appendBoundedLog(entries, {
        id,
        time: new Date(0),
        stream: 'stdout',
        text: String(id),
      })
    }
    expect(entries).toHaveLength(200)
    expect(entries[0]?.id).toBe(6)
    expect(entries.at(-1)?.id).toBe(205)
  })
})

it('computes plugin runtime utilization safely', () => {
  expect(pluginUtilization({ run_time: 0.002, expected_period: 0.01, state: 'Running' })).toBe(20)
  expect(pluginUtilization({ run_time: 1, expected_period: 0, state: 'Running' })).toBeUndefined()
})

it('keeps a valid plugin period while accepting zero runtime', () => {
  const previous = { run_time: 0.002, expected_period: 0.01, state: 'Running' }
  const merged = mergePluginStatistics(previous, {
    run_time: 0,
    expected_period: 0,
    state: 'Running',
  })

  expect(merged).toEqual({ run_time: 0, expected_period: 0.01, state: 'Running' })
  expect(pluginUtilization(merged)).toBe(0)
})

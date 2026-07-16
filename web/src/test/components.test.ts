import { mount } from '@vue/test-utils'
import { describe, expect, it } from 'vitest'
import JointsPanel from '../components/JointsPanel.vue'
import ProcessesPanel from '../components/ProcessesPanel.vue'
import StatusBadge from '../components/StatusBadge.vue'

describe('StatusBadge', () => {
  it.each([
    ['Running', 'good'],
    ['Waiting', 'warning'],
    ['Killed', 'bad'],
    ['future-state', 'neutral'],
  ])('renders %s with a text label and %s tone', (status, tone) => {
    const wrapper = mount(StatusBadge, { props: { status } })
    expect(wrapper.text()).toContain(status)
    expect(wrapper.classes()).toContain(`status-badge--${tone}`)
  })
})

describe('JointsPanel states', () => {
  it('renders an unavailable error without requiring telemetry', () => {
    const wrapper = mount(JointsPanel, {
      props: { rows: [], robotState: 'unavailable', loading: false, error: 'joint states unavailable' },
    })
    expect(wrapper.get('[role="alert"]').text()).toContain('joint states unavailable')
    expect(wrapper.text()).toContain('unavailable')
  })

  it('renders a faulted joint with its code', () => {
    const wrapper = mount(JointsPanel, {
      props: {
        rows: [{ index: 0, name: 'knee_joint', fault: 'OVER_TEMP', linkPosition: 0.2 }],
        robotState: 'live',
        loading: false,
        error: '',
      },
    })
    expect(wrapper.text()).toContain('knee_joint')
    expect(wrapper.text()).toContain('OVER_TEMP')
  })
})

describe('ProcessesPanel aggregate output', () => {
  it('orders all process lines and locally mutes one source', async () => {
    const wrapper = mount(ProcessesPanel, {
      props: {
        processes: [
          { name: 'core', status: 'Running', cmdline: {}, cmd: 'xbot2-core --hw dummy', docker: 'robot-runtime', machine: 'local', visible: true, category: 'core' },
          { name: 'camera', status: 'Running', cmdline: {}, machine: 'local', visible: true, category: 'sensors' },
        ],
        logs: {
          core: [{ id: 2, time: new Date(0), stream: 'stdout', text: 'core second' }],
          camera: [{ id: 1, time: new Date(0), stream: 'stdout', text: 'camera first' }],
        },
        loading: false,
        error: '',
      },
    })

    const console = wrapper.get('[aria-label="Aggregated read-only process output"]')
    expect(console.text()).toContain('camera first')
    expect(console.text().indexOf('camera first')).toBeLessThan(console.text().indexOf('core second'))
    const processPrefixes = console.findAll('.aggregate-log-line > strong')
    expect(processPrefixes[0]?.attributes('style')).not.toBe(processPrefixes[1]?.attributes('style'))
    expect(wrapper.get('[aria-label="core details"]').text()).toContain('xbot2-core --hw dummy')
    expect(wrapper.get('[aria-label="core details"]').text()).toContain('robot-runtime')

    const cameraToggle = wrapper.findAll(
      'input[aria-label="Mute camera in all-process output"]',
    )[0]!
    await cameraToggle.setValue(true)

    expect(console.text()).not.toContain('camera first')
    expect(console.text()).toContain('core second')
  })
})

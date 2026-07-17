import { mount } from '@vue/test-utils'
import { describe, expect, it } from 'vitest'
import JointsPanel from '../components/JointsPanel.vue'
import DiagnosticsPanel from '../components/DiagnosticsPanel.vue'
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

describe('DiagnosticsPanel', () => {
  it('renders path hierarchy, severity, values, and expansion controls', async () => {
    const status = {
      level: 1,
      name: '/Robot/xbot/joint/ankle_pitch_1/status',
      message: 'position warning',
      hardwareId: 'ankle-driver',
      values: [{ key: 'position', value: '1.2' }],
    }
    const errorStatus = {
      level: 2,
      name: '/Robot/xbot/joint/ankle_pitch_1/encoder',
      message: 'encoder fault',
      hardwareId: 'ankle-encoder',
      values: [{ key: 'code', value: '42' }],
    }
    const wrapper = mount(DiagnosticsPanel, {
      props: {
        snapshot: { stamp: 12.5, frameId: '', status: [status, errorStatus] },
        tree: [{
          label: 'Robot',
          path: '/Robot',
          level: 2,
          children: [{
            label: 'xbot',
            path: '/Robot/xbot',
            level: 2,
            children: [{
              label: 'joint',
              path: '/Robot/xbot/joint',
              level: 2,
              children: [{
                label: 'ankle_pitch_1',
                path: '/Robot/xbot/joint/ankle_pitch_1',
                level: 2,
                children: [
                  {
                    label: 'status',
                    path: status.name,
                    level: 1,
                    status,
                    children: [],
                  },
                  {
                    label: 'encoder',
                    path: errorStatus.name,
                    level: 2,
                    status: errorStatus,
                    children: [],
                  },
                ],
              }],
            }],
          }],
        }],
      },
    })

    expect(wrapper.get('[data-path="/Robot/xbot/joint/ankle_pitch_1/status"]').text())
      .toContain('position warning')
    expect(wrapper.text()).toContain('ankle-driver')
    expect(wrapper.text()).toContain('position')
    expect(wrapper.text()).toContain('1.2')
    expect(wrapper.findAll('.status-badge--warning').length).toBeGreaterThan(0)
    expect(wrapper.get('.diagnostic-alert-group--warning').text()).toContain('position warning')
    expect(wrapper.get('.diagnostic-alert-group--bad').text()).toContain('encoder fault')

    const buttons = wrapper.findAll('button')
    const expandAll = buttons.find((button) => button.text() === 'All')!
    const expandWarnings = buttons.find((button) => button.text() === 'Warn and err')!
    const expandErrors = buttons.find((button) => button.text() === 'Err')!
    const collapse = buttons.find((button) => button.text() === 'Collapse')!

    expect(wrapper.get('summary.secondary-button').text()).toBe('Expand')

    await collapse.trigger('click')
    expect(wrapper.get('[data-path="/Robot"] > details').attributes('open')).toBeDefined()
    expect(wrapper.get('[data-path="/Robot/xbot"] > details').attributes('open')).toBeDefined()
    expect(wrapper.get('[data-path="/Robot/xbot/joint"] > details').attributes('open'))
      .toBeUndefined()

    await expandAll.trigger('click')
    expect(wrapper.findAll('.diagnostic-tree details')
      .every((details) => details.attributes('open') !== undefined))
      .toBe(true)

    await collapse.trigger('click')
    await expandWarnings.trigger('click')
    expect(wrapper.get('[data-path="/Robot/xbot/joint"] > details').attributes('open'))
      .toBeDefined()

    await expandErrors.trigger('click')
    expect(wrapper.get('[data-path="/Robot"] > details').attributes('open')).toBeDefined()
    expect(wrapper.get(`[data-path="${errorStatus.name}"] > details`).attributes('open'))
      .toBeDefined()
    expect(wrapper.get(`[data-path="${status.name}"] > details`).attributes('open'))
      .toBeUndefined()
  })
})

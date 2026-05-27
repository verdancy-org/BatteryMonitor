# BatteryMonitor

Battery voltage monitor module.

## Required Hardware

- `battery_adc`
- `ramfs`

## Constructor Arguments

- `data_topic_name`: `battery_state`
- `sample_period_ms`: `50`
- `divider_ratio`: `11.0`
- `task_stack_depth`: `1024`

## Output

- `BatteryMonitor::Data`

## Notes

- Samples the board battery ADC channel.
- Applies low-pass preprocessing and derives cell count, low-voltage, and
  critical-voltage state.

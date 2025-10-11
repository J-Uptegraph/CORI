# CORI Interactive Control - Demo Guide

The smoothest way to control CORI with plain English commands!

## How It Works

1. Run `./build.bash` and select option 5
2. Wait for Gazebo to start
3. An interactive shell appears - just type naturally!

## Example Session

```
╭─────────────────────────────────────────────────╮
│  🤖 CORI INTERACTIVE CONTROL                    │
├─────────────────────────────────────────────────┤
│  Type commands in plain English!                │
│  Try: 'look left', 'wave', 'sit'               │
│  Type 'help' for all commands                  │
╰─────────────────────────────────────────────────╯

cori> look left
👈 Looking left

cori> look right
👉 Looking right

cori> wave
👋 Waving right arm

cori> sit
🪑 Sitting down

cori> stand
🧍 Standing up

cori> reset
🔄 Resetting to neutral pose

cori> quit
👋 Goodbye!
```

## All Commands

### Ultra Simple
Just type the direction:
```
cori> left
cori> right
cori> center
```

### Natural Language
Type how you'd naturally say it:
```
cori> look left
cori> look right
cori> look center

cori> wave left
cori> wave right
cori> wave              (defaults to right)

cori> reach forward
cori> reach left
cori> reach right

cori> sit
cori> stand
cori> reset
```

### Get Help
```
cori> help
```

### Exit
```
cori> quit
cori> exit
cori> q
```
Or just press Ctrl+C

## Cool Sequences to Try

### Greeting Sequence
```
cori> wave
cori> look left
cori> look right
cori> look center
cori> reset
```

### Full Body Test
```
cori> look left
cori> look right
cori> wave left
cori> wave right
cori> reach forward
cori> sit
cori> stand
cori> reset
```

## Tips

- Commands execute instantly
- You can type quickly one after another
- The robot has physics, so movements are smooth and natural
- You can still click and drag CORI in Gazebo while using the shell
- Type 'help' anytime to see all commands

## Why This Is Better

**Before:**
```bash
ros2 topic pub /model/cori/joint/head_joint/cmd_pos std_msgs/msg/Float64 '{data: 1.5}' --once
```

**Now:**
```
cori> look left
```

It's like having a conversation with your robot!

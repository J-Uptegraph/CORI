# CORI Quick Start - Interactive Control

The easiest way to control your robot!

## Start in 3 Steps

1. **Launch CORI:**
   ```bash
   ./build.bash
   ```

2. **Select option 5:**
   ```
   5) 💬 Interactive Manual Control
   ```

3. **Start typing commands!**
   ```
   cori> red
   cori> blue
   cori> wave
   cori> sit
   ```

## That's It!

No need to open new terminals, source files, or remember complex commands. Just talk to CORI naturally.

## Super Simple Commands

### 🎨 Color Commands (Look at objects on the table!)
```
red         - Look at the red object
orange      - Look at the orange object
yellow      - Look at the yellow object
green       - Look at the green object
blue        - Look at the blue object
purple      - Look at the purple object
grey/gray   - Look at the grey object
black       - Look at the black object
```

### One Word Commands
```
left        - Turn head left
right       - Turn head right
center      - Look center
wave        - Wave right arm
sit         - Sit down
stand       - Stand up
reset       - Reset pose
hi          - Wave hello
dance       - Do a dance move
```

### Two Word Commands
```
look left
look right
look center
wave left
wave right
reach forward
sit down
stand up
```

### Get Help
```
help        - Show all commands
quit        - Exit (or press Ctrl+C)
```

## Example Session

```
🎮 Starting manual control with full Gazebo integration...
🎉 CORI is ready! Starting interactive control...

╭─────────────────────────────────────────────────╮
│  🤖 CORI INTERACTIVE CONTROL                    │
├─────────────────────────────────────────────────┤
│  Type commands in plain English!                │
│  Try: 'look left', 'wave', 'sit'               │
│  Type 'help' for all commands                  │
╰─────────────────────────────────────────────────╯

cori> hi
👋 Hello! *waves*

cori> look left
👈 Looking left

cori> look right
👉 Looking right

cori> dance
💃 Let's move!
✨ Dance move complete!

cori> sit
🪑 Sitting down

cori> stand
🧍 Standing up

cori> reset
🔄 Resetting to neutral pose

cori> quit
👋 Goodbye!
```

## Cool Tricks

**Quick greeting sequence:**
```
cori> hi
cori> left
cori> right
cori> center
cori> reset
```

**Test all movements:**
```
cori> wave left
cori> wave right
cori> reach forward
cori> sit
cori> stand
cori> reset
```

**Natural conversation:**
```
cori> hello
cori> look at me
cori> wave right
cori> sit down
cori> stand up
cori> bye
```

## Why This Is Awesome

✅ **No typing ROS commands** - Just plain English
✅ **Same terminal** - Everything in one window
✅ **Instant response** - Commands execute immediately
✅ **Natural language** - Type how you think
✅ **Visual feedback** - See CORI move in Gazebo
✅ **Simple to learn** - Just start typing!

## Need More Control?

You can still:
- Click and drag CORI in Gazebo
- Use the other control options in the menu
- Connect physical hardware (ESP32)
- Use the web interface

But for quick testing and fun demos, interactive control is the way to go!

---

**Pro Tip:** Type `help` anytime to see all available commands!

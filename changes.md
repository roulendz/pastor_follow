I see **the same core problem**, just at **±2° instead of ±50°** – and your *direction* wiring now looks OK.

Here’s what jumps out:

---

### 1. You’re still hammering it with opposite commands

Over and over:

* `CMD:M, 1.96`
* then immediately `CMD:M, -1.96`
* then `CMD:M, 1.96`
* then `CMD:M, -1.96`
* …with `queue=0` almost every time.

So the host keeps saying “go +2° / now -2° / now +2° / now -2°…” many times per second.
That’s why position stays around ±2° and speed is huge (300–600°/s!), but it never settles.

This is a **control-loop / sign / logic issue on the PC side**, not a stepper hardware issue.

---

### 2. The axis direction itself looks correct now

Look near the end:

```text
FB current=-1.07° target=1.95° ...
FB current=-0.85°
FB current=-0.56°
FB current=-0.18°
FB current=0.26°
FB current=0.71°
FB current=1.09°
FB current=1.39°
FB current=1.64°
FB current=1.81° target=1.95°
```

When the **target is +1.95°** and the current is **negative**, the position moves **in the positive direction** toward the target. That’s exactly what you want. So:

* No reverse-direction button pressed ✅
* Motor + encoder / feedback sign ✅

So the **physical direction is fine**.

---

### 3. The *target* keeps flipping sign

Notice the pattern around zero:

* Current ≈ -0.3°, target = **+1.95°** → you send a **+1.96°** command
* Shortly after, current ≈ -0.1°, target = **-1.95°** → you send a **-1.96°** command

So your **target itself jumps between +1.95° and -1.95°** even though the actual position barely moved. That means:

* Either your **face→angle mapping flips sign** when the face crosses the center (or from noise),
* Or your **PID/controller chooses the wrong direction** when the error is small / noisy.

In other words, the *outer* logic is saying:

> “You’re slightly left, go hard right!”
> “Oops, now you’re slightly right, go hard left!”

…instead of “you’re near center; do nothing”.

---

### 4. No deadband, no smoothing

You’re using ±1.96° as the minimum step, and there’s:

* **No deadband:** even tiny error → full ±1.96° nudge
* **No filtering:** noise in the measured face position flips the sign of the error
* **High update rate:** new commands before the previous motion visually completes

That combination **guarantees** oscillation/chatter around the setpoint.

---

### So, what changed vs before?

* Before: It was banging around **±50–60°** – sign/mapping and scaling were way off.
* Now: It hovers around **±2°**, and the **motor direction is correct**, *but* the logic still:

  * Flips target between ±1.95°
  * Sends rapid opposing ±1.96° move commands.

You’ve fixed the **gross direction issue**, but you still need to fix the **control logic**:

1. **Add a deadband**, e.g. “if |error| < 2–3°, send no command.”
2. **Low-pass / smooth** the face position before converting to angle.
3. **Send smaller, less frequent corrections** (or reduce proportional gain) instead of full ±1.96° nudges every frame.
4. Make sure your “face is left/right of center” sign mapping is consistent and doesn’t flip just from one noisy pixel change.

If you want, next step I can help you sketch a simple pseudo-PID / deadband controller using your logs (current/target/error) so it actually *locks* around 0° instead of buzzing around it.

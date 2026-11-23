# Plan: Cart Mechanics (Kart 1.2 m × 1.0 m, 24 V)

> **Status (2026-11-23):** proposal, nothing built. It describes the first
> cart of `docs/system_plan.md` from the mechanical side: frame, suspension,
> steering, brakes, battery and where the bus nodes sit. All figures are
> estimates to check on the motor bench and on the first frame.

## 1. Goal and constraints

- A kart for children from 1.20 m body height up to adults, one vehicle
  that grows with the driver.
- **Footprint 1.20 m long × 1.00 m wide** (outside of the tyres), so it fits
  into the boot of a car without taking it apart.
- **Independent suspension** on all four wheels with some travel (about
  40 mm at the wheel).
- Built from **aluminium profiles (slot 8, item compatible) and 3D-printed
  parts**; steel only where a part carries the load alone (pins, bolts, rod
  ends).
- **Battery 24 V LiFePO4** (8S), the drives of the system plan (Jacobi,
  one per wheel), the VDC, a display, two pedals (drive, brake) and a
  mechanical brake on a Bowden cable.
- Costs within a hobby budget (about 1 k€ without motors and drives).

## 2. Main dimensions

| | Value | Note |
|---|---|---|
| Length × width | 1200 × 1000 mm | outside of the tyres, front module pushed in |
| Wheels | 6.5" hoverboard hub motor, 4.00-6 tyre (about 330 mm) on a printed clamp rim | four of them |
| Track | about 900 mm | wheel centres |
| Wheelbase | about 870 mm | 1200 mm minus one tyre diameter |
| Ground clearance | 50 to 60 mm | frame underside, unloaded |
| Mass | about 30 to 35 kg | without driver, four motors (about 3 kg each), battery about 6 kg |
| Driver reach | 0.60 to 1.10 m | seat back to pedal, 1.20 m child to about 1.90 m adult |

The square layout (track ≈ wheelbase) is fine for a kart; the low centre of
gravity (battery under the seat) matters more than the ratio.

## 3. Frame

- Two **longitudinal rails 40 × 40, 1200 mm**, about 400 mm apart; front and
  rear cross members 40 × 40 (or 40 × 80 where the suspension mounts), one
  cross member under the seat. Slot-8 profiles of a compatible supplier
  instead of original item halve the cost.
- **Battery** flat between the rails under the seat; the frame protects it
  from below with a printed or sheet skid plate.
- **Growing with the driver**, three adjustments, all on slot-nut clamps:
  - **Seat** slides about 200 mm in the rail slots.
  - **Pedal module** on its own two short profiles that slide on top of the
    rails on sliding nuts with clamping levers. It extends up to about 300 mm
    beyond the front for adults and is pushed in for transport; the front
    bumper sits on its end.
  - **Steering column** telescopic (two tubes, clamp collar), its lower
    bearing fixed at the front axle so the steering linkage never changes.
- If the whole cart is too heavy to lift, battery and seat come off with
  quick-release clamps.

## 4. Suspension

### 4.1 Rear: trailing arm per side

The hoverboard motor has a one-sided axle (flats, clamp plates), so it
clamps straight into the end of a trailing arm. The arm pivots on a profile
bracket at the frame (bushings or two deep-groove bearings on an M10 or M12
bolt), the spring strut runs from the arm to the frame.

- Simple, robust, no steering forces.
- The arm is aluminium flat bar or a short profile; the axle clamp is a
  printed or machined block with the original hoverboard clamp plates.

### 4.2 Front: sliding pillar

A vertical steel pin (20 mm, hardened shaft) is kingpin and spring guide
at once. The steering knuckle turns and slides on it on plain bushings
(iglidur or bronze); the coil spring sits on the pin between knuckle and the
upper pin mount. The motor axle clamps into the knuckle.

- Constant camber and track over the travel, very few parts, prints well.
- Damping by friction at first; a small damper or elastomer stop later if
  the cart bounces.
- **Scrub radius** about 70 to 80 mm, because the axle clamp sits on the
  inside of the wheel. Acceptable for a kart; inclining the pin a few degrees
  reduces it.
- Printed parts (knuckle, pin mounts) in PETG-CF, ASA or PC with thick
  walls; the load path runs through the pin, bolts and rod ends.

### 4.3 Springs

Cheap spring struts (e-scooter or mini ATV, 125 to 150 mm, about 15 to 25 €
each) on the rear arms; coil springs on the front pins. Travel about 40 mm at
the wheel. The spring rate is set for a mid-weight driver at first; preload
collars cover child and adult.

## 5. Steering

- Column: steel tube in printed bearing blocks, telescopic (section 3).
- Pitman arm at the lower end, two track rods from M8 threaded rod with rod
  ends to the steering arms of the knuckles. Ackermann from the angle of the
  steering arms.
- **Steering angle sensor** on the column (AS5048, as in the system plan),
  wired to the VDC. Mechanical steering only; no steering assist.

## 6. Brakes

The system plan (section 4, section 11) sets the frame: the service brake
is regenerative through the drives, and a **mechanical emergency brake must
stop the cart from top speed on its own**, independent of bus and
electronics.

- **Service brake:** the brake pedal, through the pedal unit and the VDC (or
  the limp-home path of the drives).
- **Emergency and parking brake:** a **spoon brake on both rear tyres**
  (a shoe pressed onto the tread by a lever), one Bowden cable from a hand
  lever, with a ratchet as parking brake. It works on any surface and does
  not depend on the hub motor, which has no room for a disc.
- A **ground brake** (a shoe dragged on the ground) was the first idea. It
  grips on grass and gravel but is weak and uneven on asphalt, so it cannot
  be the emergency brake; at most an addition for grass.
- To verify on the first frame: stopping distance from top speed with the
  spoon brake alone, child and adult.

## 7. Battery: 24 V LiFePO4

- 8S LiFePO4, about 20 Ah (about 500 Wh, about 6 kg): 20 V empty, 25.6 V
  nominal, 29.2 V full. Chosen over Li-ion for a children's vehicle.
- **Phase 1 without the BMS controller (Volta):** a bought pack with a smart
  BMS (UART or CAN). The VDC reads it and publishes
  `unimoc.cart.BatteryStatus` (discharge and charge limits, contactor state),
  so the drives limit the regenerative current to what the pack takes. The
  BMS controller with brake chopper of the system plan (section 5.5) follows
  later.
- **Thresholds of the drives** (settings registers, factory values are for
  the 36 V cart):

  | Register | 36 V factory | 24 V LFP proposal |
  |---|---|---|
  | `unimoc.battery.limits.regen_derate_voltage` | 45 V (Jacobi) | about 31 V |
  | `unimoc.battery.limits.trip_voltage` | 48 V (Jacobi) | about 34 V |
  | undervoltage derate start (system plan 7.2.1, planned) | 33 V | about 24 V (3.0 V per cell) |
  | undervoltage cut-off (planned) | 30 V | about 22 V (2.75 V per cell) |

  The lower overvoltage limits act as soon as a full pack refuses the charge
  current (BMS cut-off while braking). The cut-off stays well above the
  logic and gate supply limit of Jacobi (about 13 V).
- **Back-EMF and the missing chopper:** with ψ ≈ 23 mWb and 15 pole pairs
  (the factory motor) the line-to-line back-EMF peak reaches 25.6 V at about
  410 rpm, the no-load speed. On level ground the motors stay below the
  battery voltage, so after a trip the body diodes do not charge the DC link.
  Downhill above the no-load speed they do; the chopper of the BMS
  controller closes that gap later, until then the trip voltage above and the
  mechanical brake.

## 8. Drive performance (estimate)

With the factory motor (k_t 0.52 N m/A, ψ 23 mWb, 15 pole pairs) and a
330 mm tyre:

- **Top speed:** about 410 rpm no-load at 25.6 V, about 25 km/h; about
  20 km/h under load. Field weakening adds little on a surface-mount motor.
- **Traction:** 15 A per motor gives about 7.8 N m, about 47 N at the tyre.
  Two driven wheels: about 95 N, 0.7 m/s² with 135 kg (adult and cart);
  four driven wheels or 30 A peaks double that.
- To measure on the motor bench (`docs/motor_thermal_plan.md`, section 6):
  torque constant and speed at 24 V, and the heat run at the cart load.

Side note: system plan section 7.2.3 gives "about 36 V line-to-line peak at
350 rpm" for the hoverboard motor; the factory ψ gives 36 V at about
575 rpm. The gear ratio of the pedal generator depends on which one holds.

## 9. Electronics placement

| Node | Place | Note |
|---|---|---|
| Drives (Jacobi, RND 455-00370 enclosure) | on the frame next to each wheel | short phase and Hall cables; phase 1: the two rear wheels, `unimoc.vdc.drive_node_ids` without front drives, `unimoc.vdc.front_share` 0 |
| VDC (Zuse) | behind the steering column | steering angle sensor and display close by |
| Pedal unit (Drais) | on the pedal module | **own node**, not a role of the VDC: the limp-home path (system plan section 7) needs the pedals without the VDC |
| Display | on the steering column | speed, state of charge, drive mode, faults |
| Battery and BMS | under the seat | main fuse and key switch next to it |
| E-stop | pull cord and mushroom button | hard-wired loop to the enable input of every power stage |

The CAN FD harness runs along one rail; the pedal module needs a cable loop
for its travel.

## 10. Drive modes

The VDC limits speed and torque per driver (planned, `apps/vdc/`): a child
mode with a low speed limit and soft torque ramps, an adult mode with the
full range. The mode is set by a key switch or a PIN on the display, not by
the driver alone.

## 11. Cost estimate (without motors and drives)

| Item | € |
|---|---|
| Profiles and connectors | 200 to 250 |
| Spring struts, bearings, rod ends, steel pins | 150 to 200 |
| Tyres (4) | 60 to 80 |
| Seat, steering wheel | 60 to 100 |
| LiFePO4 pack with BMS | 150 to 200 |
| Pedals, display, cables, connectors | 80 to 120 |
| Filament | 60 to 80 |
| **Total** | **about 0.8 to 1.0 k** |

## 12. Order of work

1. Motor bench: torque constant, speed and heating at 24 V.
2. Frame with the rear trailing arms, two rear drives (2WD), front wheels
   unpowered on fixed pins.
3. Sliding pillar front suspension and steering with the angle sensor.
4. VDC, pedal unit, display, drive modes; the fault cases of system plan
   section 7 on a closed area.
5. Four-wheel drive, then torque vectoring.

## 13. Open questions

1. Stopping distance of the spoon brake from top speed (section 6).
2. Spring rates and whether the front needs a damper.
3. Bending of the printed knuckle and axle clamp under a curb strike;
   aluminium if the print does not hold.
4. Which smart BMS the VDC reads in phase 1, and its charge current limit.
5. Measured ψ and k_t of the actual hoverboard motors (section 8).

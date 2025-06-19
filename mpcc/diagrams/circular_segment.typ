#import "@preview/cetz:0.4.0"

#set page(width: auto, height: auto, margin: .5cm)

#let arrow-style = (stroke: (thickness: 0.5pt),
      mark: (start: "stealth", end: "stealth", fill: black),)

#cetz.canvas(
  length: 8cm,
  {
    import cetz.draw: *
    set-style(stroke: (thickness: 1pt, cap: "round"))
    line((0, 0), (1, 0), mark: (end: "stealth", fill: black))
    content((), $N$, anchor: "west")
    line((0, 0), (0, 1), mark: (end: "stealth", fill: black))
    content((), $E$, anchor: "south")
    circle((0.4, 0.8), radius: 0.005, fill: black, label: <P>)
    content((), $p$, anchor: "west", padding: 5pt)
    line((), (0, 0), mark: (start: "stealth", fill: black), stroke: (thickness: 0.8pt), name: "R")
    content((name: "R", anchor: 20%), $R$, anchor: "west", padding: 5pt)
    arc(
      (name: "R", anchor: 80%),
      start: calc.atan2(0.4, 0.8),
      delta: -calc.atan2(0.4, 0.8),
      radius: 0.179,
      ..arrow-style,
      name: "phi-angle"
    )
    content((name: "phi-angle", anchor: 30%), $phi$, anchor: "west", padding: 5pt)
    arc(
      (0.5, 0),
      start: 0deg,
      delta: 20deg,
      radius: 0.5,
      name: "xi-angle",
      ..arrow-style
    )
    content("xi-angle.mid", $xi$, anchor: "west", padding: 5pt)
    rotate(20deg)
    line((0, 0), (0.6, 0), stroke: (dash: "dashed"))
    arc((0.5, 0), start: 0deg, delta: 80deg, radius: 0.5)
  },
)

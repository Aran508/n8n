---
name: portfolio-builder
description: Build a 9-page portfolio from the user's resume that shows how they think, not just what they did. Each project gets a Problem / Approach / Where I lost marks / Result / Learnt breakdown. Use when the user says "build my portfolio", "portfolio", or starts stage 1 of the job-hunt route.
argument-hint: "[target role, e.g. 'mechanical engineering intern' or 'frontend developer']"
user-invocable: true
---

# /portfolio-builder

A resume says *what* someone did. A portfolio shows *how they think*. This skill turns the user's resume and project notes into a 9-page portfolio.

## Inputs
1. **Resume.** Look in `career/resume.*` first. If it isn't there, ask the user to upload it. Read the whole thing.
2. **Target role.** Use the argument if one was given; otherwise ask.
3. **Project raw material.** For the 2–3 strongest projects, ask a few questions at a time (use AskUserQuestion when the options are clear):
   - What was the brief, and what constraints or limits did it have?
   - Why did you pick that approach, and what did you have to teach yourself?
   - What went wrong, or where did you lose marks, and why?
   - What was the measurable result (grade, users, speed, cost, a deliverable)?
   - What would you change next time?
   Never invent numbers, grades or outcomes. If the user doesn't know, leave a clearly marked `[TODO: …]`.

## The 9 pages
1. **Cover**: name, target role, one-line value statement, contact links.
2. **About me**: three or four sentences on who they are and what they're aiming for, plus a photo placeholder.
3. **Skills map**: tools and skills grouped by area, each tied to the project that proves it.
4. **Projects overview**: one card per project with a thumbnail placeholder and a one-line outcome.
5. **Project deep dive #1**: Problem → Approach → Where I lost marks → Result → Learnt.
6. **Project deep dive #2**: the same structure.
7. **Project deep dive #3**, or work experience if there are only two strong projects: the same structure.
8. **Extracurriculars and leadership**: what they did and what it taught them.
9. **Closing**: what they want next, contact details, QR/link placeholder.

"Where I lost marks" is the page that sets this portfolio apart. Keep it honest and specific, and always pair it with what was learnt.

## Output
- Write `career/portfolio/portfolio.md` as the source content.
- Build `career/portfolio/portfolio.html`: one printable A4 page per section, using `@page { size: A4 }` and `page-break-after`. Make the design clean and consistent, with one accent colour and readable type.
- Render a PDF with Playwright (`page.pdf({format:'A4', printBackground:true})`) to `career/portfolio/portfolio.pdf`, and screenshot pages 1 and 5 to check the layout.
- Offer to publish the HTML as a private Artifact so the user gets a shareable link.
- List every remaining `[TODO]` for the user to fill in.

## Then
Tell the user stage 1 is done and the next step is `/resume-panel`.

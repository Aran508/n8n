---
name: resume-panel
description: Run the user's resume past a simulated 7-person hiring panel (ATS bot, recruiter, HR, hiring manager, team lead, peer engineer, skeptic). Each reviewer votes HIRE or REJECT with reasons, then the resume is rewritten until no reviewer rejects it. Use for "review my resume", "resume panel", "make my resume bulletproof", or stage 2 of the job-hunt route.
argument-hint: "[target role or a pasted job description]"
user-invocable: true
---

# /resume-panel

Different readers reject a resume for different reasons. This skill simulates the whole panel so every objection surfaces before a real one does.

## Inputs
- Resume: `career/resume.*`. If it isn't there, ask for it.
- Target: a job description (best), or else a role title. If `career/jobs/shortlist.md` exists, offer to tailor the resume to a specific job on it.
- Portfolio, if built: `career/portfolio/portfolio.md`, which can supply evidence for stronger bullets.

## The panel
Each reviewer reads the resume in character and returns **HIRE / REJECT**, the top three problems (quote the exact line), and a fix for each.

1. **ATS bot**: keyword match against the job description, parseable layout (no tables, columns or images for text), standard section headings, file type. Give a match score out of 100 and list the missing keywords.
2. **Recruiter (6-second scan)**: what they take away from the top third, title fit, gaps, and length.
3. **HR**: dates, consistency, professionalism, location and eligibility signals.
4. **Hiring manager**: impact. Does each bullet show a result (a number, scale, or outcome) instead of a duty?
5. **Team lead**: technical depth. Are the skills claimed actually backed by projects?
6. **Peer engineer**: would they want to work with this person? Do the projects sound real and owned?
7. **Skeptic**: buzzwords, vague claims, anything that sounds inflated or unverifiable.

## Loop
1. Show the scoreboard: `REJECTS: n / 7`, with one line per reviewer.
2. Merge the fixes into one rewritten resume. **Never invent experience, metrics, employers or dates.** Where a metric would help but isn't known, ask the user or insert `[add number]`.
3. Run the panel again on the new version. Stop at 0 rejects or after 3 rounds, whichever comes first, and list the objections still open.

## Output
- `career/resume-v2.md`, the final text, plus a clean single-column ATS-safe `career/resume-v2.html` and, via Playwright, `career/resume-v2.pdf`.
- `career/resume-panel-report.md`: each round's votes and the changes made.
- When tailored to one job, write the files as `career/tailored/<company>-<role>.md` instead.

## Then
Next stage: `/job-finder`.

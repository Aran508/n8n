---
name: apply-assistant
description: Prepare job applications from the shortlist. Drafts tailored answers to each form question, fills web application forms in a browser, and stops before submitting so the user reviews and clicks Submit. Tracks status in a pipeline file. Use for "apply to jobs", "fill applications", or stage 4 of the job-hunt route.
argument-hint: "[number of jobs from the shortlist, or a job link]"
user-invocable: true
---

# /apply-assistant

## Hard rules
- **Never click Submit or Apply, and never send anything on the user's behalf.** Fill the form, then stop and hand over: "N ready for you to review."
- Never invent facts. Every answer must trace back to the resume, the portfolio, or something the user said.
- Never create accounts, solve CAPTCHAs, or enter passwords. If a site requires a login, mark the job `needs-user` and move on.
- Ask once for the details that forms usually need (full name, email, phone, location, work authorisation, notice period, expected salary, LinkedIn, GitHub, portfolio link) and save them to `career/profile.md`. Ask before reusing anything sensitive.

## Steps
1. Pick jobs from `career/jobs/shortlist.md`: the top N, or the ones the user names. Skip any already in `career/applications.md`.
2. For each job:
   - Use the tailored resume `career/tailored/<company>-<role>.*` if there is one; otherwise offer to run `/resume-panel` for that job first.
   - Draft the answers: "Why us?" (2–3 sentences tying one real project to the company's product), the cover letter if one is asked for, and the screening questions. Save them to `career/applications/<company>-<role>.md`.
3. Form filling, where the environment can reach the site: open the posting with Playwright (headed only if the user can watch), fill the fields and attach the resume PDF, take a screenshot, and **stop before Submit**. If the site is blocked or the form can't be automated, provide the copy-paste answer pack instead.
4. Update `career/applications.md`: Company | Role | Link | Status (`drafted` / `filled-awaiting-user` / `submitted-by-user` / `needs-user`) | Date | Follow-up date (+7 days).

## Then
Remind the user to submit, then offer: follow-up email drafts after 7 days, and interview prep for any replies.

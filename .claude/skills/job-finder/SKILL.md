---
name: job-finder
description: Find fresh jobs posted directly on company career pages (Greenhouse, Lever, Ashby, Workable and others) rather than crowded LinkedIn listings. Searches with targeted queries, scores each job against the user's resume, and keeps a deduplicated shortlist. Use for "find jobs", "job search", or stage 3 of the job-hunt route.
argument-hint: "<role> [location|remote] [e.g. 'frontend engineer remote']"
user-invocable: true
---

# /job-finder

Many companies post on their own applicant-tracking-system boards before (or instead of) LinkedIn, and applying early there gets a much better response rate.

## Inputs
- Role keywords, location or remote, seniority (intern / new grad / junior), and any must-haves (visa, salary, stack). Ask for whatever is missing in a single question.
- Resume: `career/resume-v2.md` if it exists, otherwise `career/resume.*`.

## Search
Use WebSearch (load it with ToolSearch first). Run these queries in parallel, one batch per role variant:
```
site:boards.greenhouse.io "<role>" <location/remote>
site:jobs.lever.co "<role>" <location/remote>
site:jobs.ashbyhq.com "<role>" <location/remote>
site:apply.workable.com "<role>" <location/remote>
site:jobs.smartrecruiters.com "<role>" <location/remote>
"<role>" "new grad" OR "graduate" OR "entry level" careers <location>
```
For India-based searches, also try `site:naukri.com`, `site:wellfound.com` and `site:instahyre.com`.
Prefer postings from the last 14 days. Open the promising postings (with the web-fetch agent if there's no WebFetch tool) to confirm they're still open and to read the requirements.

If the network blocks a job site, say which host was blocked and carry on with the sources that work. Never fill the list with invented jobs.

## Score
For each job, score fit from 0 to 100 against the resume: required skills matched, seniority, location and eligibility, and recency. Note the one or two gaps.

## Output
- `career/jobs/shortlist.md`: a table with Score | Company | Role | Location | Posted | Link | Gaps, sorted by score. Merge with any existing list and drop duplicates by link.
- Show the top 10 in chat.

## Then
Suggest: `/resume-panel <job link>` to tailor the resume for the top picks, then `/apply-assistant`.

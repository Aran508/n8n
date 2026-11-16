# Job-hunt route

Upload your resume as `career/resume.pdf` (or .docx/.md), then run the stages in order.

| Stage | When | Command | Output |
|---|---|---|---|
| 0. Setup | Once | Upload your resume | `career/resume.*` |
| 1. Portfolio | First | `/portfolio-builder <target role>` | `career/portfolio/portfolio.pdf` |
| 2. Resume panel | After the portfolio | `/resume-panel <target role>` | `career/resume-v2.pdf` (0/7 rejects) |
| 3. Find jobs | Once the resume passes | `/job-finder <role> <location/remote>` | `career/jobs/shortlist.md` |
| 4. Tailor | For each top job | `/resume-panel <job link>` | `career/tailored/<company>-<role>.pdf` |
| 5. Apply | After tailoring | `/apply-assistant <N>` | Forms filled; **you** click Submit |
| 6. Learn from a video | Any time | `/watch <video file or link> [question]` | Summary of a tutorial or reel |

Repeat stages 3–5 every few days; new postings get the most replies in their first week.

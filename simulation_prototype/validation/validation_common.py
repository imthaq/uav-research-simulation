import os


class Checker:
    def __init__(self):
        self.results = []

    def check(self, task, description, condition, detail=""):
        passed = bool(condition)
        self.results.append({"task": task, "description": description, "passed": passed, "detail": detail})
        status = "PASS" if passed else "FAIL"
        print(f"[{status}] {task}: {description}" + (f" ({detail})" if detail else ""))
        return passed

    def close(self, a, b, tol=1e-6):
        return abs(a - b) <= tol

    def summary(self):
        total = len(self.results)
        passed = sum(1 for r in self.results if r["passed"])
        by_task = {}
        for r in self.results:
            by_task.setdefault(r["task"], []).append(r["passed"])
        return total, passed, by_task

    def print_summary(self):
        total, passed, by_task = self.summary()
        failed = total - passed
        print("\n=== Summary by task ===")
        for task, outcomes in by_task.items():
            print(f"  {task}: {sum(outcomes)}/{len(outcomes)} passed")
        print(f"\nTotal: {passed}/{total} checks passed" + (f", {failed} FAILED" if failed else ""))
        return failed

    def write_markdown(self, path, title, intro=""):
        total, passed, by_task = self.summary()
        failed = total - passed
        lines = [f"# {title}", ""]
        if intro:
            lines += [intro, ""]
        lines += [f"**Result: {passed}/{total} checks passed**"
                   + (f" — {failed} FAILED" if failed else " — all green"), ""]
        lines += ["## Summary by task", "", "| Task | Passed |", "|---|---|"]
        for task, outcomes in by_task.items():
            lines.append(f"| {task} | {sum(outcomes)}/{len(outcomes)} |")
        lines += ["", "## Detailed results", ""]
        lines += ["| Status | Task | Description | Detail |", "|---|---|---|---|"]
        for r in self.results:
            status = "PASS" if r["passed"] else "**FAIL**"
            detail = str(r["detail"]).replace("|", "\\|")
            desc = str(r["description"]).replace("|", "\\|")
            lines.append(f"| {status} | {r['task']} | {desc} | {detail} |")
        lines.append("")
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            f.write("\n".join(lines))
        return path

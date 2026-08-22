#!/usr/bin/env python3
"""Compare two recorded two-book Stage-1 experiments."""

import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys


CORE_PHASE = "stage1_two_book_core"
PHASES = (
    "book_pick_place_total",
    "pick_frame_zero_prepare",
    "pick_alignment",
    "pick_book_vision",
    "pick_alignment_motion",
    "pick_replay",
    "cart_coarse_navigation",
    "place_alignment",
    "place_observation_pose",
    "place_cart_vision",
    "place_alignment_motion",
    "place_replay",
    "table_return",
)


def _timing_path(value):
    path = Path(value)
    return path / "timings.jsonl" if path.is_dir() else path


def _load(value):
    path = _timing_path(value)
    records = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as error:
                raise ValueError(f"{path}:{line_number}: {error}") from error
    return path, records


def _records(records, phase):
    return [record for record in records if record.get("phase") == phase]


def _single_success(records, phase):
    matches = _records(records, phase)
    if len(matches) != 1 or matches[0].get("ok") is not True:
        return None
    return matches[0]


def _quality(records):
    problems = []
    total = _single_success(records, "stage1_total")
    if total is None:
        problems.append("缺少唯一且成功的 stage1_total")
    core = _single_success(records, CORE_PHASE)
    if core is None:
        problems.append("缺少唯一且成功的 stage1_two_book_core")

    books = _records(records, "book_pick_place_total")
    book_indices = [record.get("book_index") for record in books]
    if len(book_indices) != 2 or set(book_indices) != {1, 2}:
        problems.append("两本书总阶段记录不完整")
    elif any(record.get("ok") is not True for record in books):
        problems.append("至少一本 Pick→Place 失败")

    pick_alignments = _records(records, "pick_alignment")
    if len(pick_alignments) != 2 or any(
        record.get("ok") is not True
        or record.get("xy_within_tolerance") is not True
        for record in pick_alignments
    ):
        problems.append("Pick 精定位未全部通过原门限")

    place_alignments = _records(records, "place_alignment")
    if len(place_alignments) != 2 or any(
        record.get("ok") is not True
        or record.get("within_tolerance") is not True
        for record in place_alignments
    ):
        problems.append("Place 精定位未全部通过原门限")

    pick_replays = _records(records, "pick_replay")
    if len(pick_replays) != 2 or any(
        record.get("ok") is not True
        or record.get("frames_sent") != 333
        or record.get("d01_holding") is False
        for record in pick_replays
    ):
        problems.append("Pick 未全部完整回放 333 帧或 D01 明确未吸住")

    place_replays = _records(records, "place_replay")
    if len(place_replays) != 2 or any(
        record.get("ok") is not True
        or record.get("frames_sent") != 388
        or record.get("d01_released") is not True
        for record in place_replays
    ):
        problems.append("Place 未全部完整回放 388 帧并确认释放")

    corrections = {
        "pick": len(_records(records, "pick_alignment_motion")),
        "place": len(_records(records, "place_alignment_motion")),
    }
    vision_calls = {
        "pick": len(_records(records, "pick_book_vision")),
        "place": len(_records(records, "place_cart_vision")),
    }
    return {
        "ok": not problems,
        "problems": problems,
        "core_s": None if core is None else float(core["duration_s"]),
        "configuration": None if total is None else {
            "book_count": total.get("book_count"),
            "skip_initial_coarse": total.get("skip_initial_coarse"),
            "book_align_mode": total.get("book_align_mode"),
            "book_coarse": total.get("book_coarse"),
            "book_pick_press_mm": total.get("book_pick_press_mm"),
        },
        "pick_holding": {
            record.get("book_index"): record.get("d01_holding")
            for record in pick_replays
        },
        "corrections": corrections,
        "vision_calls": vision_calls,
    }


def _phase_totals(records):
    totals = defaultdict(float)
    by_book = defaultdict(lambda: defaultdict(float))
    for record in records:
        phase = record.get("phase")
        if phase not in PHASES or record.get("ok") is not True:
            continue
        duration = float(record.get("duration_s", 0.0))
        totals[phase] += duration
        book_index = record.get("book_index")
        if book_index in (1, 2):
            by_book[int(book_index)][phase] += duration
    return totals, by_book


def _metadata(records):
    for record in records:
        if record.get("phase") == CORE_PHASE:
            return {
                "run_id": record.get("run_id"),
                "commit": record.get("commit"),
            }
    return {"run_id": None, "commit": None}


def _format_delta(baseline, candidate):
    delta = candidate - baseline
    percent = 0.0 if baseline == 0.0 else delta / baseline * 100.0
    return f"{baseline:8.2f} -> {candidate:8.2f} s  {delta:+8.2f} s ({percent:+6.1f}%)"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", help="基线实验目录或 timings.jsonl")
    parser.add_argument("candidate", help="候选实验目录或 timings.jsonl")
    args = parser.parse_args()

    baseline_path, baseline_records = _load(args.baseline)
    candidate_path, candidate_records = _load(args.candidate)
    baseline_quality = _quality(baseline_records)
    candidate_quality = _quality(candidate_records)
    baseline_meta = _metadata(baseline_records)
    candidate_meta = _metadata(candidate_records)

    print(
        "基线: "
        f"run={baseline_meta['run_id']} commit={baseline_meta['commit']} "
        f"file={baseline_path}"
    )
    print(
        "候选: "
        f"run={candidate_meta['run_id']} commit={candidate_meta['commit']} "
        f"file={candidate_path}"
    )

    if not baseline_quality["ok"]:
        print("结论: 基线无效，不能比较")
        for problem in baseline_quality["problems"]:
            print(f"  - {problem}")
        return 2

    if not candidate_quality["ok"]:
        print("结论: 有害（候选没有通过质量门禁）")
        for problem in candidate_quality["problems"]:
            print(f"  - {problem}")
        return 1

    if baseline_quality["configuration"] != candidate_quality["configuration"]:
        print("结论: 实验配置不同，不能比较")
        print(f"  基线配置: {baseline_quality['configuration']}")
        print(f"  候选配置: {candidate_quality['configuration']}")
        return 2

    baseline_core = baseline_quality["core_s"]
    candidate_core = candidate_quality["core_s"]
    print("核心两本:", _format_delta(baseline_core, candidate_core))

    baseline_totals, baseline_books = _phase_totals(baseline_records)
    candidate_totals, candidate_books = _phase_totals(candidate_records)
    print("分阶段:")
    for phase in PHASES:
        baseline_value = baseline_totals.get(phase, 0.0)
        candidate_value = candidate_totals.get(phase, 0.0)
        if baseline_value or candidate_value:
            print(f"  {phase:28s} {_format_delta(baseline_value, candidate_value)}")
    print("逐本总时间:")
    for book_index in (1, 2):
        baseline_value = baseline_books[book_index].get(
            "book_pick_place_total", 0.0
        )
        candidate_value = candidate_books[book_index].get(
            "book_pick_place_total", 0.0
        )
        print(
            f"  book {book_index}: "
            f"{_format_delta(baseline_value, candidate_value)}"
        )

    print(
        "修正次数: "
        f"Pick {baseline_quality['corrections']['pick']} -> "
        f"{candidate_quality['corrections']['pick']}; "
        f"Place {baseline_quality['corrections']['place']} -> "
        f"{candidate_quality['corrections']['place']}"
    )
    print(
        "视觉调用: "
        f"Pick {baseline_quality['vision_calls']['pick']} -> "
        f"{candidate_quality['vision_calls']['pick']}; "
        f"Place {baseline_quality['vision_calls']['place']} -> "
        f"{candidate_quality['vision_calls']['place']}"
    )

    corrections_increased = any(
        candidate_quality["corrections"][name]
        > baseline_quality["corrections"][name]
        for name in ("pick", "place")
    )
    vision_calls_increased = any(
        candidate_quality["vision_calls"][name]
        > baseline_quality["vision_calls"][name]
        for name in ("pick", "place")
    )
    holding_degraded = any(
        baseline_quality["pick_holding"].get(book_index) is True
        and candidate_quality["pick_holding"].get(book_index) is not True
        for book_index in (1, 2)
    )
    delta_s = candidate_core - baseline_core
    delta_ratio = delta_s / baseline_core
    required_improvement_s = max(10.0, baseline_core * 0.05)
    if (
        corrections_increased
        or vision_calls_increased
        or holding_degraded
        or delta_ratio > 0.05
    ):
        print("结论: 有害（质量退化、修正/视觉调用增加或耗时增加超过 5%）")
        return 1
    if -delta_s >= required_improvement_s:
        print("结论: 初步有益；再做一组配对实验并比较中位数后再正式保留")
        return 0
    print("结论: 证据不足（核心耗时变化未超过 max(10 s, 5%)）")
    return 3


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print(f"比较失败: {error}", file=sys.stderr)
        sys.exit(2)

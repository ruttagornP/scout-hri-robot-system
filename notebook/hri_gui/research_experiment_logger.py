#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import threading
from datetime import datetime


class ResearchExperimentLogger:
    """Reusable, thread-safe CSV logger for HRI/robotics experiments."""

    DEFAULT_FIELDS = [
        "timestamp", "trial_id", "mode", "language", "transcript",
        "expected_command", "predicted_command", "confidence", "result",
        "recording_duration_s", "processing_latency_s", "noise_condition",
        "speaker_id", "distance_m", "notes",
    ]

    def __init__(self, csv_path="research_experiment.csv", fieldnames=None):
        self.csv_path = os.path.abspath(csv_path)
        self.fieldnames = fieldnames or self.DEFAULT_FIELDS
        self._lock = threading.Lock()
        self.trial_counter = self._get_next_trial_id()

    def _get_next_trial_id(self):
        if not os.path.exists(self.csv_path):
            return 1
        try:
            with open(self.csv_path, "r", encoding="utf-8-sig", newline="") as f:
                rows = list(csv.DictReader(f))
            ids = []
            for row in rows:
                try:
                    ids.append(int(row.get("trial_id", 0)))
                except (TypeError, ValueError):
                    pass
            return max(ids, default=0) + 1
        except Exception:
            return 1

    def _ensure_file(self):
        directory = os.path.dirname(self.csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        if not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0:
            with open(self.csv_path, "w", encoding="utf-8-sig", newline="") as f:
                csv.DictWriter(
                    f, fieldnames=self.fieldnames, extrasaction="ignore"
                ).writeheader()

    def log_trial(
        self, *, mode="ONLY VOICE", language="", transcript="",
        expected_command="", predicted_command="", confidence=0.0,
        result="", recording_duration_s=0.0, processing_latency_s=0.0,
        noise_condition="", speaker_id="", distance_m="", notes="",
        trial_id=None
    ):
        with self._lock:
            self._ensure_file()
            if trial_id is None:
                trial_id = self.trial_counter
                self.trial_counter += 1

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "trial_id": trial_id,
                "mode": mode,
                "language": language,
                "transcript": transcript,
                "expected_command": expected_command,
                "predicted_command": predicted_command,
                "confidence": f"{float(confidence):.4f}",
                "result": result,
                "recording_duration_s": f"{float(recording_duration_s):.3f}",
                "processing_latency_s": f"{float(processing_latency_s):.3f}",
                "noise_condition": noise_condition,
                "speaker_id": speaker_id,
                "distance_m": distance_m,
                "notes": notes,
            }

            with open(self.csv_path, "a", encoding="utf-8-sig", newline="") as f:
                csv.DictWriter(
                    f, fieldnames=self.fieldnames, extrasaction="ignore"
                ).writerow(row)

            return trial_id

    def log(self, **kwargs):
        return self.log_trial(**kwargs)

    def get_path(self):
        return self.csv_path

##############################################################
#    Logger for Proposed system (B1)                         #
##############################################################
# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# """
# Reusable research experiment CSV logger.

# Designed to be copied into other HRI/robotics projects.
# It has no dependency on PyQt, ROS 2, Whisper, OpenCV, etc.
# """

# import csv
# import os
# import threading
# from datetime import datetime


# class ResearchExperimentLogger:
#     """Thread-safe CSV logger for repeatable research experiments."""

#     DEFAULT_FIELDS = [
#         "timestamp",
#         "trial_id",
#         "experiment_id",
#         "mode",
#         "language",
#         "speaker_id",
#         "expected_command",
#         "transcript",
#         "predicted_command",
#         "confidence",
#         "result",
#         "recording_duration_s",
#         "asr_latency_s",
#         "classification_latency_s",
#         "total_latency_s",
#         "noise_condition",
#         "distance_m",
#         "notes",
#     ]

#     def __init__(self, csv_path="research_experiment.csv", experiment_id="EXP001"):
#         self.csv_path = os.path.abspath(csv_path)
#         self.experiment_id = experiment_id
#         self.fieldnames = list(self.DEFAULT_FIELDS)
#         self._lock = threading.Lock()
#         self.trial_counter = self._get_next_trial_id()

#     def _get_next_trial_id(self):
#         if not os.path.exists(self.csv_path):
#             return 1
#         try:
#             with open(self.csv_path, "r", encoding="utf-8-sig", newline="") as f:
#                 rows = csv.DictReader(f)
#                 ids = []
#                 for row in rows:
#                     try:
#                         ids.append(int(row.get("trial_id", 0)))
#                     except (TypeError, ValueError):
#                         pass
#                 return max(ids, default=0) + 1
#         except Exception:
#             return 1

#     def _ensure_file(self):
#         directory = os.path.dirname(self.csv_path)
#         if directory:
#             os.makedirs(directory, exist_ok=True)

#         if not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0:
#             with open(self.csv_path, "w", encoding="utf-8-sig", newline="") as f:
#                 csv.DictWriter(
#                     f,
#                     fieldnames=self.fieldnames,
#                     extrasaction="ignore",
#                 ).writeheader()

#     def log_trial(
#         self,
#         *,
#         mode="ONLY VOICE",
#         language="",
#         speaker_id="",
#         expected_command="",
#         transcript="",
#         predicted_command="",
#         confidence=0.0,
#         result="",
#         recording_duration_s=0.0,
#         asr_latency_s=0.0,
#         classification_latency_s=0.0,
#         total_latency_s=0.0,
#         noise_condition="",
#         distance_m="",
#         notes="",
#         experiment_id=None,
#     ):
#         with self._lock:
#             self._ensure_file()

#             trial_id = self.trial_counter
#             self.trial_counter += 1

#             row = {
#                 "timestamp": datetime.now().isoformat(timespec="seconds"),
#                 "trial_id": trial_id,
#                 "experiment_id": experiment_id or self.experiment_id,
#                 "mode": mode,
#                 "language": language,
#                 "speaker_id": speaker_id,
#                 "expected_command": expected_command,
#                 "transcript": transcript,
#                 "predicted_command": predicted_command,
#                 "confidence": f"{float(confidence):.4f}",
#                 "result": result,
#                 "recording_duration_s": f"{float(recording_duration_s):.3f}",
#                 "asr_latency_s": f"{float(asr_latency_s):.3f}",
#                 "classification_latency_s": f"{float(classification_latency_s):.3f}",
#                 "total_latency_s": f"{float(total_latency_s):.3f}",
#                 "noise_condition": noise_condition,
#                 "distance_m": distance_m,
#                 "notes": notes,
#             }

#             with open(self.csv_path, "a", encoding="utf-8-sig", newline="") as f:
#                 csv.DictWriter(
#                     f,
#                     fieldnames=self.fieldnames,
#                     extrasaction="ignore",
#                 ).writerow(row)

#             return trial_id

#     def get_path(self):
#         return self.csv_path

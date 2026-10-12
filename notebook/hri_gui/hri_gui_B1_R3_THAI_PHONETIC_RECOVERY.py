#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
hri_gui.py
===========================================================
HRI GUI - ONLY VOICE + B1 R3 THAI PHONETIC RECOVERY

Design:
    1. Robot video: UDP/RTP JPEG UNICAST only
    2. Notebook microphone -> Faster-Whisper -> text
    3. Text -> robot command
    4. Robot command is published as std_msgs/String on:
           /voice_command
    5. GUI contains exactly 3 HRI mode buttons:
           ONLY VOICE
           ONLY GESTURE
           VOICE + GESTURE
       In this first version ONLY VOICE is implemented.
       ONLY GESTURE and VOICE + GESTURE are placeholders.

Network:
    ROBOT_IP       = 10.42.0.185
    VIDEO_PORT     = 5000
    AUDIO_RX_PORT  = 5001
    AUDIO_TX_PORT  = 6000

IMPORTANT:
    AUDIO_RX_PORT and AUDIO_TX_PORT are kept as configuration
    because they are part of the existing robot system. This
    first ONLY VOICE implementation does NOT stream raw audio
    to port 6000. It captures the notebook microphone locally,
    performs speech recognition, and sends the recognized
    command through ROS 2.

    The robot must have a ROS 2 subscriber to /voice_command
    if you want the recognized command to control the robot.

Expected command strings:
    FORWARD
    BACKWARD
    LEFT
    RIGHT
    STOP
"""

import sys
import time
import re
import threading
import queue
from difflib import SequenceMatcher

from research_experiment_logger import ResearchExperimentLogger

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8

from PyQt5.QtCore import (
    Qt,
    QThread,
    pyqtSignal,
)
from PyQt5.QtGui import (
    QImage,
    QPixmap,
    QFont,
)
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QTextEdit,
    QComboBox,
    QMessageBox,
    QScrollArea,
)

# GStreamer through OpenCV.
# OpenCV must be built with GStreamer support.
# The pipeline itself is handled by cv2.VideoCapture.
#
# Faster-Whisper is imported lazily so the GUI can still start
# and report a useful error if it has not been installed.


# ============================================================
# USER CONFIGURATION
# ============================================================

ROBOT_IP = "10.42.0.185"

VIDEO_PORT = 5000
AUDIO_RX_PORT = 5001
AUDIO_TX_PORT = 6000

# ROS 2 topic used by this GUI to send recognized HRI commands.
VOICE_COMMAND_TOPIC = "/voice_command"
CONTROL_MODE_TOPIC = "/control_mode"
# 0 = JOY, 1 = HUMAN FOLLOW, 2 = VOICE

# Research experiment logging
RESEARCH_LOG_CSV = "research_experiment.csv"

# Video settings. These must match the robot RTP/JPEG sender.
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
VIDEO_FPS = 15

# Whisper settings
WHISPER_MODEL = "small"
WHISPER_LANGUAGE = "th"

# R3: Thai-only recognition. The experiment contains exactly five Thai commands.
AUTO_DETECT_LANGUAGE = False

# Microphone capture
SAMPLE_RATE = 16000
CHANNELS = 1

# Voice activity / recording
# The first implementation uses a push-to-talk button.
# This is much more reliable for a research prototype than
# always-on speech detection.
RECORD_CHUNK_SECONDS = 0.10
MAX_RECORD_SECONDS = 5.0

# ============================================================
# COMMAND DICTIONARY
# ============================================================

# ============================================================
# ENGLISH ASR CORRECTION (B1)
# Only targeted corrections are added on top of the baseline.
# No threshold, fuzzy matching, Thai logic, or robot architecture
# is changed in this version.
# ============================================================

ENGLISH_ASR_CORRECTIONS = {
    # FORWARD
    "forwad": "forward",
    "foward": "forward",
    "forword": "forward",

    # BACKWARD
    "backwad": "backward",
    "backword": "backward",
    "backwards": "backward",

    # LEFT
    "lef": "left",

    # RIGHT
    "rite": "right",
    "righ": "right",
    "go lai": "go right",
    "go live": "go right",
    "go like": "go right",
    "alright": "right",
    "all right": "right",

    # STOP
    "stopp": "stop",
}



# ============================================================
# THAI ASR CORRECTION + MATCHING (B1)
# English recognition/matching is intentionally left unchanged.
# This block is used only when the recognized text contains Thai.
# ============================================================

THAI_ASR_CORRECTIONS = {
    # Forward
    "เดินน้า": "เดินหน้า",
    "เดินนา": "เดินหน้า",
    "เดินหน้าา": "เดินหน้า",

    # Backward
    "ขอยหลัง": "ถอยหลัง",
    "ไหว หลัง": "ถอยหลัง",
    "ไหวหลัง": "ถอยหลัง",

    # Left
    # Existing corrections are preserved.
    "เรียวซ้าย": "เลี้ยวซ้าย",
    "เฮียวซ้าย": "เลี้ยวซ้าย",

    # Targeted variants observed in the Thai LEFT trials.
    "เหลี่ยวซ้าย": "เลี้ยวซ้าย",
    "เรียงซ้าย": "เลี้ยวซ้าย",
    "เมียวซ้าย": "เลี้ยวซ้าย",
    "กิดซ้าย": "เลี้ยวซ้าย",
    "เยี่ยมซ้าย": "เลี้ยวซ้าย",

    # Additional LEFT variants observed in the latest trials.
    "เรียวสาย": "เลี้ยวซ้าย",
    "เรียบสาย": "เลี้ยวซ้าย",
    "เรียวไฟ": "เลี้ยวซ้าย",

    # R2 holdout-recovery aliases from the R1 error analysis.
    "เรียวซัย": "เลี้ยวซ้าย",
    "เดี๋ยวสาย": "เลี้ยวซ้าย",
    "เนียวสาย": "เลี้ยวซ้าย",

    # Right
    # Existing corrections are preserved.
    "เรียวขวา": "เลี้ยวขวา",
    "เรียวหวะ": "เลี้ยวขวา",
    "เหลียวหวะ": "เลี้ยวขวา",

    # Targeted variants observed in the Thai RIGHT trials.
    "รีวขวา": "เลี้ยวขวา",
    "ลี่ยวหว่า": "เลี้ยวขวา",
    "เรียวหว่า": "เลี้ยวขวา",
    "เหลียวฝ่า": "เลี้ยวขวา",
    "เหลือฝ่า": "เลี้ยวขวา",
    "เหลือหวา": "เลี้ยวขวา",
    "เรียมฝ่า": "เลี้ยวขวา",
    "เรียวหวาน": "เลี้ยวขวา",

    # Additional RIGHT variants observed in the latest trials.

    # R2 holdout-recovery alias from the R1 error analysis.
    "เมียวหวะ": "เลี้ยวขวา",

    # Stop
    # Targeted STOP ASR corrections requested from test results.
    "ยุด": "หยุด",
    "มันยุด": "หยุด",
    "หยุดด": "หยุด",
    "หยุดๆ": "หยุด",
}

# Thai-only phrases.  These are deliberately kept separate from
# the English matching path so that the English algorithm is unchanged.
THAI_COMMAND_KEYWORDS = {
    "FORWARD": [
        "ไปข้างหน้า",
        "เดินหน้า",
        "ตรงไป",
    ],
    "BACKWARD": [
        "ถอยหลัง",
        "เดินถอยหลัง",
    ],
    "LEFT": [
        "เลี้ยวซ้าย",
        "ซ้าย",
    ],
    "RIGHT": [
        "เลี้ยวขวา",
        "ขวา",
    ],
    "STOP": [
        "หยุด",
        "หยุดรถ",
    ],
}

THAI_FUZZY_THRESHOLD = 0.70

# ============================================================
# R3 THAI PHONETIC MATCHING
# ============================================================
# R2 showed that the remaining LEFT/RIGHT errors are mainly
# phonetic ASR distortions, e.g.:
#   ลี่โอซ่าย -> เลี้ยวซ้าย
#   เบียวหว่า  -> เลี้ยวขวา
#   เวียวหว่า  -> เลี้ยวขวา
#
# These are handled by command-specific phonetic normalization,
# NOT by globally lowering the fuzzy threshold.
# ============================================================

R3_PHONETIC_THRESHOLD = 0.72
R3_PHONETIC_MARGIN = 0.08

# Generic pronunciation variants for the "เลี้ยว" sound.
# They are used only inside the phonetic scoring function.
R3_TURN_VARIANTS = (
    "เลี้ยว",
    "เรียว",
    "เหลียว",
    "เฮียว",
    "เดี๋ยว",
    "เนียว",
    "เมียว",
    "เบียว",
    "เวียว",
    "ลี่โอ",
    "ลิโอ",
)

# Direction endings observed as Thai ASR spelling variations.
R3_LEFT_ENDING_VARIANTS = (
    "ซ้าย",
    "ซัย",
    "สาย",
    "ซาย",
    "ส้าย",
    "ซ่าย",
)

R3_RIGHT_ENDING_VARIANTS = (
    "ขวา",
    "หวา",
    "หว่า",
    "หวะ",
    "ฝ่า",
    "หวาน",
)

def r3_phonetic_normalize(text, command=None):
    """
    Command-aware phonetic normalization.

    Important:
      - This is NOT a global replacement.
      - Directional replacements are applied only for the
        corresponding candidate command.
      - This prevents unrelated words from becoming commands.
    """
    t = normalize_thai_text(text)

    if not t:
        return ""

    # Normalize common variants of the "เลี้ยว" sound.
    for variant in sorted(R3_TURN_VARIANTS, key=len, reverse=True):
        if variant in t:
            t = t.replace(variant, "เลี้ยว")

    if command == "LEFT":
        for variant in sorted(R3_LEFT_ENDING_VARIANTS, key=len, reverse=True):
            if variant in t:
                t = t.replace(variant, "ซ้าย")

    elif command == "RIGHT":
        for variant in sorted(R3_RIGHT_ENDING_VARIANTS, key=len, reverse=True):
            if variant in t:
                t = t.replace(variant, "ขวา")

    elif command == "STOP":
        # Conservative STOP normalization.
        # "ยุด" is close to "หยุด"; very short "ยุ" is NOT
        # normalized because it is too ambiguous by itself.
        if t == "ยุด":
            t = "หยุด"

    return normalize_thai_text(t)


def r3_phonetic_similarity(text, phrase, command):
    """
    Return a command-specific phonetic similarity score [0,1].
    """
    normalized_text = normalize_thai_text(text)
    normalized_phrase = normalize_thai_text(phrase)

    if not normalized_text or not normalized_phrase:
        return 0.0

    normalized_text = r3_phonetic_normalize(
        normalized_text, command
    )
    normalized_phrase = r3_phonetic_normalize(
        normalized_phrase, command
    )

    if normalized_text == normalized_phrase:
        return 1.0

    return SequenceMatcher(
        None,
        normalized_text,
        normalized_phrase
    ).ratio()


def r3_command_score(text, command, phrases):
    """
    Weighted R3 score:
        45% character similarity
        55% command-aware phonetic similarity
    """
    best_edit = 0.0
    best_phonetic = 0.0

    normalized = normalize_thai_text(text)

    for phrase in phrases:
        p = normalize_thai_text(phrase)

        edit_score = thai_text_similarity(normalized, p)
        phonetic_score = r3_phonetic_similarity(
            normalized,
            p,
            command
        )

        best_edit = max(best_edit, edit_score)
        best_phonetic = max(best_phonetic, phonetic_score)

    score = (
        0.45 * best_edit +
        0.55 * best_phonetic
    )

    return score, best_edit, best_phonetic


def r3_command_grammar(text, command):
    """
    Lightweight command grammar.

    It does not invent commands. It only gives a small bonus when
    the transcript contains evidence consistent with the candidate.
    """
    t = normalize_thai_text(text)

    if command == "LEFT":
        if any(x in t for x in (
            "ซ้าย", "ซัย", "สาย", "ซาย", "ส้าย", "ซ่าย",
            "เลี้ยว", "เรียว", "เหลียว", "เดี๋ยว",
            "เนียว", "เมียว", "เบียว", "เวียว"
        )):
            return 1.0

    elif command == "RIGHT":
        if any(x in t for x in (
            "ขวา", "หวา", "หว่า", "หวะ", "ฝ่า",
            "หวาน", "เลี้ยว", "เรียว", "เหลียว",
            "เดี๋ยว", "เนียว", "เมียว", "เบียว", "เวียว"
        )):
            return 1.0

    elif command == "FORWARD":
        if any(x in t for x in ("เดินหน้า", "ไปข้างหน้า", "ตรงไป")):
            return 1.0

    elif command == "BACKWARD":
        if any(x in t for x in (
            "ถอยหลัง",
            "เดินถอยหลัง",
            "ภายหลัง",
            "ขอยหลัง",
            "ไหวหลัง",
        )):
            return 1.0

    elif command == "STOP":
        # STOP grammar is deliberately strict.
        if any(x in t for x in STOP_RESCUE_PHRASES):
            return 1.0

    return 0.0

# ============================================================
# B1 R3 STOP ASR RESCUE
# ============================================================
# Used only when the normal Thai ASR/classifier does not
# recognize a command. We do NOT map unrelated Thai words
# (e.g. เจ้า, ดู, เกิด) directly to STOP.
#
# The rescue pass re-runs Whisper with a Thai STOP-focused
# prompt, then accepts STOP only when the second-pass text
# contains an observed STOP form.
# ============================================================

STOP_RESCUE_PHRASES = (
    "หยุด",
    "ยุด",
    "มันยุด",
    "หยุดด",
    "หยุดๆ",
)

STOP_RESCUE_MIN_LOGPROB = -1.0


def contains_thai(text):
    """Return True when text contains at least one Thai Unicode character."""
    return bool(re.search(r"[\u0E00-\u0E7F]", text or ""))


def normalize_thai_text(text):
    """Normalize Thai ASR text for Thai command matching only."""
    if not text:
        return ""

    text = text.strip().lower()

    # Remove punctuation commonly produced by Whisper.
    text = re.sub(r"[.,!?;:，。！？；：]+", "", text)

    # Collapse whitespace.
    text = re.sub(r"\s+", " ", text)

    return text.strip()


def correct_thai_asr(text):
    """Apply targeted Thai ASR corrections only."""
    normalized = normalize_thai_text(text)

    if not normalized:
        return ""

    # Longer phrases first to avoid partial replacement problems.
    corrections = sorted(
        THAI_ASR_CORRECTIONS.items(),
        key=lambda item: len(item[0]),
        reverse=True,
    )

    for wrong, correct in corrections:
        wrong_n = normalize_thai_text(wrong)
        correct_n = normalize_thai_text(correct)

        if wrong_n:
            normalized = normalized.replace(wrong_n, correct_n)

    return normalize_thai_text(normalized)


def thai_text_similarity(a, b):
    """String similarity used only by the Thai recognition path."""
    return SequenceMatcher(None, a, b).ratio()


def thai_command_from_text(text):
    """
    R3 Thai command classifier.

    Order:
        1. Thai ASR correction
        2. Explicit STOP safety priority
        3. Exact / substring matching
        4. Weighted edit + phonetic matching
        5. Grammar consistency
        6. Reject ambiguous candidates

    The classifier still returns ONLY the five robot commands.
    """
    normalized = correct_thai_asr(text)

    if not normalized:
        return None, 0.0

    # --------------------------------------------------------
    # STOP safety: explicit evidence only.
    # --------------------------------------------------------
    for phrase in THAI_COMMAND_KEYWORDS["STOP"]:
        p = normalize_thai_text(phrase)
        if p and p in normalized:
            return "STOP", 1.0

    # --------------------------------------------------------
    # Exact / substring matching.
    # --------------------------------------------------------
    for command, phrases in THAI_COMMAND_KEYWORDS.items():
        if command == "STOP":
            continue

        for phrase in phrases:
            p = normalize_thai_text(phrase)

            if normalized == p:
                return command, 1.0

            # Keep the existing high-confidence behavior.
            if p in normalized:
                return command, 0.90

    # --------------------------------------------------------
    # R3 weighted phonetic matching.
    # --------------------------------------------------------
    candidates = []

    for command, phrases in THAI_COMMAND_KEYWORDS.items():
        if command == "STOP":
            # STOP is handled conservatively below.
            continue

        score, edit_score, phonetic_score = r3_command_score(
            normalized,
            command,
            phrases
        )

        grammar = r3_command_grammar(
            normalized,
            command
        )

        # Grammar bonus is intentionally small.
        final_score = score + (0.05 * grammar)

        candidates.append(
            (
                final_score,
                command,
                edit_score,
                phonetic_score,
                grammar,
            )
        )

    candidates.sort(reverse=True, key=lambda x: x[0])

    if not candidates:
        return None, 0.0

    best = candidates[0]

    if len(candidates) > 1:
        second = candidates[1]
        margin = best[0] - second[0]
    else:
        margin = 1.0

    # Require both a sufficiently strong score and separation
    # from the second-best command.
    # For fuzzy recovery, require command-consistent evidence.
    # This prevents a short unrelated word such as "หลัง" from
    # being accepted as BACKWARD only because edit similarity is high.
    if (
        best[0] >= R3_PHONETIC_THRESHOLD
        and margin >= R3_PHONETIC_MARGIN
        and best[4] >= 1.0
    ):
        return best[1], min(best[0], 1.0)

    return None, best[0]


COMMAND_KEYWORDS = {
    "FORWARD": [
        "forward",
        "go forward",
        "move forward",
        "ahead",
        "ไปข้างหน้า",
        "เดินหน้า",
        "ตรงไป",
    ],
    "BACKWARD": [
        "backward",
        "go backward",
        "move backward",
        "back",
        "reverse",
        "ถอยหลัง",
        "เดินถอยหลัง",
    ],
    "LEFT": [
        "left",
        "turn left",
        "go left",
        "move left",
        "เลี้ยวซ้าย",
        "ซ้าย",
    ],
    "RIGHT": [
        "right",
        "turn right",
        "go right",
        "move right",
        "เลี้ยวขวา",
        "ขวา",
    ],
    "STOP": [
        "stop",
        "halt",
        "freeze",
        "หยุด",
        "หยุดรถ",
    ],
}


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def normalize_text(text):
    """Normalize recognized speech for keyword matching."""
    if not text:
        return ""

    text = text.strip().lower()

    # Collapse repeated whitespace.
    text = re.sub(r"\s+", " ", text)

    return text


def correct_english_asr(text):
    """Apply only known English ASR corrections before baseline matching."""
    normalized = normalize_text(text)
    if not normalized:
        return ""

    # Phrase correction first (e.g. "go lai" -> "go right").
    for wrong, correct in ENGLISH_ASR_CORRECTIONS.items():
        wrong_n = normalize_text(wrong)
        correct_n = normalize_text(correct)
        if " " in wrong_n:
            normalized = normalized.replace(wrong_n, correct_n)
        else:
            normalized = re.sub(
                rf"\b{re.escape(wrong_n)}\b",
                correct_n,
                normalized,
            )

    return normalize_text(normalized)


def is_stop_rescue_text(text):
    """Return True only for explicit/observed Thai STOP forms."""
    normalized = normalize_thai_text(text)
    corrected = correct_thai_asr(normalized)

    for phrase in STOP_RESCUE_PHRASES:
        p = normalize_thai_text(phrase)
        if p and p in corrected:
            return True, corrected

    return False, corrected


def stop_asr_rescue(self, audio):
    """
    R3 STOP rescue.

    A second Whisper pass is used only when normal classification
    fails. STOP is accepted only when:
      1. Whisper produces an explicit/phonetic STOP candidate,
      2. phonetic score is above threshold,
      3. the candidate is sufficiently separated from ambiguity,
      4. ASR average log probability is acceptable.

    Unrelated words such as ดู, เจ้า, ด้วย, เยี่ยม are NOT
    hard-coded as STOP.
    """
    if self.model is None:
        return False, "", 0.0

    try:
        self.status_changed.emit(
            "STOP Rescue R3: second Whisper pass..."
        )

        segments, info = self.model.transcribe(
            audio,
            language="th",
            beam_size=10,
            best_of=1,
            temperature=0.0,
            vad_filter=False,
            condition_on_previous_text=False,
            initial_prompt=(
                "คำสั่งหุ่นยนต์: หยุด หยุดรถ ยุด"
            ),
            no_speech_threshold=0.30,
            compression_ratio_threshold=2.4,
            log_prob_threshold=-1.0
        )

        rescue_parts = []
        log_probs = []

        for segment in segments:
            rescue_parts.append(segment.text)

            if getattr(segment, "avg_logprob", None) is not None:
                log_probs.append(float(segment.avg_logprob))

        rescue_text = " ".join(rescue_parts).strip()

        if not rescue_text:
            self.status_changed.emit(
                "STOP Rescue R3: no speech"
            )
            return False, "", 0.0

        avg_logprob = (
            sum(log_probs) / len(log_probs)
            if log_probs else 0.0
        )

        corrected = correct_thai_asr(rescue_text)

        # Explicit STOP forms remain highest priority.
        explicit_stop = False
        for phrase in STOP_RESCUE_PHRASES:
            p = normalize_thai_text(phrase)
            if p and p in corrected:
                explicit_stop = True
                break

        # Phonetic score against STOP.
        _, edit_score, phonetic_score = r3_command_score(
            corrected,
            "STOP",
            THAI_COMMAND_KEYWORDS["STOP"]
        )

        stop_score = max(
            phonetic_score,
            0.45 * edit_score + 0.55 * phonetic_score
        )

        self.status_changed.emit(
            f"STOP Rescue R3: {rescue_text} "
            f"| corrected={corrected} "
            f"| score={stop_score:.2f} "
            f"| logprob={avg_logprob:.2f}"
        )

        # Explicit forms are accepted with acceptable ASR confidence.
        if (
            explicit_stop
            and avg_logprob >= STOP_RESCUE_MIN_LOGPROB
        ):
            self.status_changed.emit(
                "STOP Rescue R3: ACCEPTED (explicit STOP)"
            )
            return True, rescue_text, avg_logprob

        # Otherwise require strong phonetic evidence.
        if (
            stop_score >= 0.78
            and avg_logprob >= STOP_RESCUE_MIN_LOGPROB
        ):
            self.status_changed.emit(
                "STOP Rescue R3: ACCEPTED (phonetic STOP)"
            )
            return True, rescue_text, avg_logprob

        self.status_changed.emit(
            "STOP Rescue R3: REJECTED"
        )
        return False, rescue_text, avg_logprob

    except Exception as exc:
        self.status_changed.emit(
            f"STOP Rescue R3 error: {exc}"
        )
        return False, "", 0.0


def is_repetitive_asr_hallucination(text):
    """
    Detect obviously repetitive Whisper output before command matching.
    Only very long text with repeated short chunks is rejected.
    """
    normalized = normalize_thai_text(text)
    if len(normalized) < 60:
        return False
    words = normalized.split()
    if len(words) < 12:
        return False
    for n in (2, 3, 4):
        chunks = [" ".join(words[i:i+n]) for i in range(len(words)-n+1)]
        counts = {}
        for chunk in chunks:
            counts[chunk] = counts.get(chunk, 0) + 1
        if max(counts.values(), default=0) >= 4:
            return True
    return False


def command_from_text(text):
    """
    Convert speech text into one of the 5 robot commands.

    IMPORTANT:
        - English path below is intentionally unchanged.
        - Thai text is routed to the Thai-only algorithm above.

    Returns:
        (command, confidence_score)
    """
    if not text:
        return None, 0.0

    # --------------------------------------------------------
    # THAI PATH
    # --------------------------------------------------------
    # If Whisper produced Thai text, do NOT run the English
    # correction/token algorithm. Use the dedicated Thai path.
    if contains_thai(text):
        # ----------------------------------------------------
        # STOP PRIORITY (B1.3)
        # ----------------------------------------------------
        # Safety rule: when a Thai transcript contains a
        # recognized STOP phrase, STOP has priority over any
        # movement command in the same utterance.
        #
        # Examples:
        #     "ขวา หยุด"       -> STOP
        #     "ซ้าย หยุด"       -> STOP
        #     "เดินหน้า หยุด"  -> STOP
        #     "ขวา ขวา หยุด"   -> STOP
        #
        # This does NOT map unrelated words to STOP and does
        # NOT change the normal Thai fuzzy-matching algorithm.
        normalized_thai = correct_thai_asr(text)
        stop_phrases = THAI_COMMAND_KEYWORDS.get("STOP", [])

        for stop_phrase in stop_phrases:
            stop_n = normalize_thai_text(stop_phrase)
            if stop_n and stop_n in normalized_thai:
                return "STOP", 1.0

        return thai_command_from_text(text)

    # --------------------------------------------------------
    # ENGLISH PATH — UNCHANGED
    # --------------------------------------------------------
    normalized = correct_english_asr(text)

    if not normalized:
        return None, 0.0

    # Exact / phrase matching first.
    for command, phrases in COMMAND_KEYWORDS.items():
        for phrase in phrases:
            p = normalize_text(phrase)

            if normalized == p:
                return command, 1.0

            if p in normalized:
                return command, 0.90

    # Token-based fallback for English.
    tokens = set(normalized.split())

    fallback = {
        "FORWARD": {"forward", "ahead"},
        "BACKWARD": {"backward", "reverse", "back"},
        "LEFT": {"left"},
        "RIGHT": {"right"},
        "STOP": {"stop", "halt", "freeze"},
    }

    for command, words in fallback.items():
        if tokens.intersection(words):
            return command, 0.75

    return None, 0.0


# ============================================================
# ROS 2 NODE
# ============================================================

# B1 R3 Thai experiment: phonetic matching + grammar + STOP safety.

class HRICommandNode(Node):

    def __init__(self):
        super().__init__("hri_voice_gui")

        self.voice_command_pub = self.create_publisher(
            String,
            VOICE_COMMAND_TOPIC,
            10
        )

        self.control_mode_pub = self.create_publisher(
            UInt8,
            CONTROL_MODE_TOPIC,
            10
        )

        self.get_logger().info(
            f"HRI voice command publisher ready: "
            f"{VOICE_COMMAND_TOPIC}"
        )

    def publish_control_mode(self, mode):
        msg = UInt8()
        msg.data = int(mode)
        self.control_mode_pub.publish(msg)
        self.get_logger().info(
            f"CONTROL MODE -> {int(mode)}"
        )

    def publish_command(self, command):
        msg = String()
        msg.data = command

        self.voice_command_pub.publish(msg)

        self.get_logger().info(
            f"VOICE COMMAND -> {command}"
        )


# ============================================================
# ROS 2 SPIN THREAD
# ============================================================

class ROS2Thread(QThread):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True

    def run(self):
        while self.running and rclpy.ok():
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1
            )

    def stop(self):
        self.running = False


# ============================================================
# VIDEO THREAD
# ============================================================

class VideoThread(QThread):

    frame_received = pyqtSignal(object)
    status_changed = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self.running = True
        self.cap = None

        self.last_frame_time = time.monotonic()
        self.frame_count = 0
        self.fps = 0.0
        self.fps_start = time.monotonic()

    def build_pipeline(self):
        """
        UNICAST ONLY.

        RTP JPEG:
            UDP -> RTP JPEG depay -> JPEG decode -> BGR -> appsink
        """

        caps = (
            "application/x-rtp,"
            "media=video,"
            "clock-rate=90000,"
            "encoding-name=JPEG,"
            "payload=26"
        )

        pipeline = (
            f"udpsrc "
            f"address=0.0.0.0 "
            f"port={VIDEO_PORT} "
            f"caps=\"{caps}\" ! "
            "rtpjpegdepay ! "
            "jpegdec ! "
            "videoconvert ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        return pipeline

    def run(self):
        pipeline = self.build_pipeline()

        print("\n[VIDEO] UNICAST ONLY")
        print("[VIDEO] Robot IP:", ROBOT_IP)
        print("[VIDEO] Video port:", VIDEO_PORT)
        print("[VIDEO] Pipeline:")
        print(pipeline)

        self.status_changed.emit(
            f"Video: starting UNICAST UDP:{VIDEO_PORT}"
        )

        try:
            self.cap = cv2.VideoCapture(
                pipeline,
                cv2.CAP_GSTREAMER
            )
        except Exception as exc:
            print("[VIDEO] OpenCV/GStreamer error:", exc)
            self.status_changed.emit(
                "Video: ERROR opening GStreamer"
            )
            return

        if not self.cap.isOpened():
            print(
                "[VIDEO] ERROR: Cannot open GStreamer pipeline"
            )
            print(
                "[VIDEO] Check OpenCV GStreamer support."
            )
            self.status_changed.emit(
                "Video: ERROR - pipeline not opened"
            )
            return

        self.status_changed.emit(
            "Video: CONNECTED - UNICAST"
        )

        while self.running:

            ok, frame = self.cap.read()

            if not ok:
                time.sleep(0.01)
                continue

            self.last_frame_time = time.monotonic()
            self.frame_count += 1

            elapsed = time.monotonic() - self.fps_start

            if elapsed >= 1.0:
                self.fps = (
                    self.frame_count / elapsed
                )
                self.frame_count = 0
                self.fps_start = time.monotonic()

            self.frame_received.emit(frame)

        self.release()

    def release(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

            self.cap = None

    def stop(self):
        self.running = False
        self.release()


# ============================================================
# VOICE THREAD
# ============================================================

class VoiceThread(QThread):

    status_changed = pyqtSignal(str)
    transcript_ready = pyqtSignal(str, str, float)
    error_signal = pyqtSignal(str)
    ready_signal = pyqtSignal(bool)

    def __init__(self):
        super().__init__()

        self.running = True
        self.recording = False

        self.audio_queue = queue.Queue()

        self.model = None
        self.recognizer_lock = threading.Lock()

        self.stop_record_event = threading.Event()
        self.language = None

    def load_model(self):

        try:
            from faster_whisper import WhisperModel
        except ImportError:
            self.error_signal.emit(
                "ไม่พบ faster-whisper\n\n"
                "ติดตั้งด้วย:\n"
                "pip install faster-whisper"
            )
            return False

        try:
            self.status_changed.emit(
                f"Loading Whisper model: {WHISPER_MODEL}"
            )

            # CPU configuration is intentionally conservative.
            # Users with CUDA can change this to:
            # device="cuda", compute_type="float16"
            self.model = WhisperModel(
                WHISPER_MODEL,
                device="cpu",
                compute_type="int8"
            )

            self.status_changed.emit(
                "Whisper: READY"
            )
            self.ready_signal.emit(True)

            return True

        except Exception as exc:
            self.ready_signal.emit(False)
            self.error_signal.emit(
                f"Whisper model error:\n{exc}"
            )
            return False

    def record_from_microphone(self):

        try:
            import sounddevice as sd
        except ImportError:
            self.error_signal.emit(
                "ไม่พบ sounddevice\n\n"
                "ติดตั้งด้วย:\n"
                "pip install sounddevice"
            )
            return None

        audio_data = []

        # Clear the stop flag only when a NEW recording actually starts.
        self.stop_record_event.clear()
        self.recording = True

        self.status_changed.emit(
            "Microphone: LISTENING..."
        )

        def callback(indata, frames, callback_time, status):
            if status:
                print("[MIC]", status)
            if self.running and self.recording:
                audio_data.append(indata.copy())

        try:
            with sd.InputStream(
                samplerate=SAMPLE_RATE,
                channels=CHANNELS,
                dtype="float32",
                blocksize=int(SAMPLE_RATE * RECORD_CHUNK_SECONDS),
                callback=callback
            ):
                start_time = time.monotonic()

                while self.running and not self.stop_record_event.is_set():
                    if time.monotonic() - start_time >= MAX_RECORD_SECONDS:
                        self.status_changed.emit(
                            "Microphone: maximum recording time reached"
                        )
                        break
                    time.sleep(0.01)

        except Exception as exc:
            self.recording = False
            self.error_signal.emit(
                f"Microphone error:\n{exc}"
            )
            return None
        finally:
            self.recording = False

        if not audio_data:
            return None

        audio = np.concatenate(audio_data, axis=0).flatten()

        # Remove DC offset.
        audio = audio - np.mean(audio)

        # Your ALSA default device produces values above +/-1.0.
        # Normalize only when necessary.
        peak = float(np.max(np.abs(audio)))
        if peak > 1.0:
            audio = audio / peak
            self.status_changed.emit(
                f"Microphone: normalized (peak={peak:.2f})"
            )

        rms = float(np.sqrt(np.mean(audio ** 2)))
        self.status_changed.emit(
            f"Microphone: RMS={rms:.3f}"
        )

        return audio

    def transcribe(self, audio):

        if self.model is None:
            return

        try:
            self.status_changed.emit(
                "Whisper: TRANSCRIBING..."
            )

            segments, info = self.model.transcribe(
                audio,
                language="th",
                beam_size=10,
                best_of=1,
                temperature=0.0,
                vad_filter=True,
                vad_parameters={
                    "min_silence_duration_ms": 100
                },
                condition_on_previous_text=False,
                initial_prompt=(
                    "Robot commands: forward, backward, left, right, stop."
                    if self.language != "th"
                    else "คำสั่งหุ่นยนต์: เดินหน้า ถอยหลัง ซ้าย ขวา หยุด"
                ),
                no_speech_threshold=0.8,
                compression_ratio_threshold=2.4,
                log_prob_threshold=-1.0
            )

            text_parts = []
            for segment in segments:
                text_parts.append(segment.text)

            text = " ".join(text_parts).strip()

            if is_repetitive_asr_hallucination(text):
                self.status_changed.emit(
                    "Whisper: repetitive ASR output rejected"
                )
                self.transcript_ready.emit(
                    text, "UNKNOWN", 0.0
                )
                return

            if not text:
                # ------------------------------------------------
                # NO-SPEECH RECOVERY
                # ------------------------------------------------
                # Push-to-talk already defines the recording window.
                # If the first Whisper pass returns an empty transcript,
                # retry once WITHOUT VAD. Short Thai commands can be
                # removed by VAD even when the microphone RMS is high.
                self.status_changed.emit(
                    "Whisper: no text -> retrying without VAD..."
                )

                retry_segments, retry_info = self.model.transcribe(
                    audio,
                    language="th",
                    beam_size=10,
                    best_of=1,
                    temperature=0.0,
                    vad_filter=False,
                    condition_on_previous_text=False,
                    initial_prompt=(
                        "คำสั่งหุ่นยนต์: เดินหน้า ถอยหลัง ซ้าย ขวา หยุด"
                    ),
                    no_speech_threshold=0.30,
                    compression_ratio_threshold=2.4,
                    log_prob_threshold=-1.0
                )

                retry_parts = []
                for segment in retry_segments:
                    retry_parts.append(segment.text)

                retry_text = " ".join(retry_parts).strip()

                if retry_text:
                    text = retry_text
                    self.status_changed.emit(
                        f"Whisper: no-speech recovery -> {text}"
                    )
                else:
                    # IMPORTANT FOR RESEARCH:
                    # No-speech remains a valid experimental outcome.
                    self.status_changed.emit(
                        "Whisper: no speech detected after recovery"
                    )
                    self.transcript_ready.emit(
                        "", "NO_SPEECH", 0.0
                    )
                    return

            command, score = command_from_text(text)

            # ----------------------------------------------------
            # B1 R3 STOP ASR RESCUE
            # ----------------------------------------------------
            # Only run when the normal classifier did not find a
            # command. This preserves the existing Thai algorithm
            # and uses a second Whisper pass only for recovery.
            if command is None and contains_thai(text):
                rescued, rescue_text, rescue_logprob = (
                    stop_asr_rescue(self, audio)
                )

                if rescued:
                    command = "STOP"
                    score = 1.0
                    self.status_changed.emit(
                        f"STOP Rescue override: "
                        f"{text} -> STOP "
                        f"(rescue='{rescue_text}', "
                        f"logprob={rescue_logprob:.2f})"
                    )

            if contains_thai(text):
                corrected = correct_thai_asr(text)
                if corrected != normalize_thai_text(text):
                    self.status_changed.emit(
                        f"Thai ASR correction: {text} -> {corrected}"
                    )

            if command is None:
                self.status_changed.emit(
                    "Voice: command not recognized"
                )
                self.transcript_ready.emit(
                    text, "UNKNOWN", 0.0
                )
                return

            self.transcript_ready.emit(
                text, command, score
            )

            self.status_changed.emit(
                f"Voice: {command}"
            )

        except Exception as exc:
            self.error_signal.emit(
                f"Whisper error:\n{exc}"
            )

    def run(self):

        if not self.load_model():
            return

        self.status_changed.emit(
            "Voice system READY"
        )

        while self.running:

            # Wait until GUI places a recording request.
            try:
                request = self.audio_queue.get(
                    timeout=0.1
                )
            except queue.Empty:
                continue

            if request != "RECORD":
                continue

            audio = self.record_from_microphone()

            if audio is None:
                continue

            if len(audio) < SAMPLE_RATE * 0.2:
                # Treat an unusably short capture as a no-speech trial
                # so it is not silently lost from the research dataset.
                self.status_changed.emit(
                    "Whisper: no speech detected (recording too short)"
                )
                self.transcript_ready.emit(
                    "", "NO_SPEECH", 0.0
                )
                continue

            self.transcribe(audio)

        self.status_changed.emit(
            "Voice: STOPPED"
        )

    def start_recording(self):

        if not self.isRunning() or self.model is None:
            return

        if self.recording:
            return

        self.stop_record_event.clear()

        try:
            self.audio_queue.put_nowait("RECORD")
        except queue.Full:
            pass

    def stop_recording(self):

        if self.recording:
            self.stop_record_event.set()

    def stop(self):

        self.running = False
        self.stop_record_event.set()

        try:
            self.audio_queue.put_nowait(
                "STOP"
            )
        except queue.Full:
            pass


# ============================================================
# MAIN GUI
# ============================================================

class HRIGUI(QMainWindow):

    def __init__(self, ros_node):

        super().__init__()

        self.ros_node = ros_node

        self.setWindowTitle(
            "Scout Robot - HRI GUI"
        )

        self.resize(
            1200,
            700
        )

        self.voice_thread = None
        self.video_thread = None

        self.current_mode = "ONLY VOICE"

        self.last_command = "NONE"
        self.last_transcript = ""

        # Reusable research logger, independent of ROS/Whisper/PyQt.
        self.experiment_logger = ResearchExperimentLogger(RESEARCH_LOG_CSV)

        self.init_ui()

        # GUI starts in ONLY VOICE, so synchronize Robot Joy Relay.
        self.set_robot_control_mode(2)

        # Start Whisper loading immediately so the first push-to-talk
        # press cannot race with model initialization.
        self.ensure_voice_thread()
        self.start_video()

    # --------------------------------------------------------
    # UI
    # --------------------------------------------------------

    def init_ui(self):

        central = QWidget()
        self.setCentralWidget(central)

        outer_layout = QVBoxLayout(central)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        content = QWidget()
        root = QVBoxLayout(content)
        root.setContentsMargins(10, 5, 10, 10)
        root.setSpacing(6)

        scroll.setWidget(content)
        outer_layout.addWidget(scroll)

        # ====================================================
        # TITLE
        # ====================================================

        title = QLabel(
            "SCOUT ROBOT — HUMAN ROBOT INTERACTION"
        )

        title.setAlignment(
            Qt.AlignCenter
        )

        title.setFont(
            QFont(
                "Arial",
                16,
                QFont.Bold
            )
        )

        root.addWidget(title)

        # ====================================================
        # MODE SELECTION
        # ====================================================

        mode_group = QGroupBox(
            "HRI MODE"
        )

        mode_layout = QHBoxLayout()

        self.btn_voice_mode = QPushButton(
            "ONLY VOICE"
        )

        self.btn_gesture_mode = QPushButton(
            "ONLY GESTURE"
        )

        self.btn_fusion_mode = QPushButton(
            "VOICE + GESTURE / FUSION"
        )

        self.btn_voice_mode.clicked.connect(
            lambda: self.select_mode(
                "ONLY VOICE"
            )
        )

        self.btn_gesture_mode.clicked.connect(
            lambda: self.select_mode(
                "ONLY GESTURE"
            )
        )

        self.btn_fusion_mode.clicked.connect(
            lambda: self.select_mode(
                "VOICE + GESTURE / FUSION"
            )
        )

        mode_layout.addWidget(
            self.btn_voice_mode
        )

        mode_layout.addWidget(
            self.btn_gesture_mode
        )

        mode_layout.addWidget(
            self.btn_fusion_mode
        )

        mode_group.setLayout(
            mode_layout
        )

        root.addWidget(
            mode_group
        )

        self.update_mode_buttons()

        # ====================================================
        # MAIN CAMERA
        # ====================================================

        camera_group = QGroupBox(
            "ROBOT CAMERA — UNICAST ONLY"
        )

        camera_layout = QVBoxLayout()

        self.video_label = QLabel(
            "Waiting for robot video..."
        )

        self.video_label.setAlignment(
            Qt.AlignCenter
        )

        self.video_label.setMinimumSize(
            800,
            320
        )

        self.video_label.setStyleSheet(
            "background-color: #111;"
            "color: white;"
            "font-size: 18px;"
        )

        camera_layout.addWidget(
            self.video_label
        )

        self.video_status = QLabel(
            "Video: starting..."
        )

        camera_layout.addWidget(
            self.video_status
        )

        camera_group.setLayout(
            camera_layout
        )

        root.addWidget(
            camera_group,
            stretch=1
        )

        # ====================================================
        # VOICE CONTROL
        # ====================================================

        voice_group = QGroupBox(
            "ONLY VOICE CONTROL"
        )

        voice_layout = QVBoxLayout()

        info = QLabel(
            "Notebook microphone → Whisper → "
            "Command classification → ROS 2 /voice_command → "
            "Research CSV logger"
        )

        info.setWordWrap(
            True
        )

        voice_layout.addWidget(
            info
        )

        self.voice_status = QLabel(
            "Voice: OFF"
        )

        voice_layout.addWidget(
            self.voice_status
        )

        self.btn_voice_start = QPushButton(
            "🎤 PRESS AND HOLD TO SPEAK"
        )

        self.btn_voice_start.setMinimumHeight(
            48
        )
        self.btn_voice_start.setEnabled(False)

        self.btn_voice_start.pressed.connect(
            self.start_voice
        )

        self.btn_voice_start.released.connect(
            self.stop_voice
        )

        voice_layout.addWidget(
            self.btn_voice_start
        )

        # ----------------------------------------------------
        # R3: Thai-only
        # ----------------------------------------------------

        lang_label = QLabel(
            "Whisper language: Thai (TH)"
        )

        voice_layout.addWidget(
            lang_label
        )

        voice_group.setLayout(
            voice_layout
        )

        root.addWidget(
            voice_group
        )

        # ====================================================
        # RESEARCH EXPERIMENT LOGGER
        # ====================================================

        research_group = QGroupBox("RESEARCH EXPERIMENT LOGGER")
        research_layout = QVBoxLayout()

        expected_layout = QHBoxLayout()
        expected_layout.addWidget(QLabel("Expected command:"))

        self.expected_command_combo = QComboBox()
        self.expected_command_combo.addItems([
            "NOT SET",
            "FORWARD",
            "BACKWARD",
            "LEFT",
            "RIGHT",
            "STOP",
            "NONE / NON-COMMAND",
        ])
        expected_layout.addWidget(self.expected_command_combo)
        expected_layout.addStretch()
        research_layout.addLayout(expected_layout)

        noise_layout = QHBoxLayout()
        noise_layout.addWidget(QLabel("Noise:"))

        self.noise_combo = QComboBox()
        self.noise_combo.addItems([
            "NOT SET",
            "QUIET",
            "LOW",
            "MEDIUM",
            "HIGH",
        ])
        noise_layout.addWidget(self.noise_combo)
        noise_layout.addStretch()
        research_layout.addLayout(noise_layout)

        self.logger_status = QLabel(
            f"CSV: {self.experiment_logger.get_path()}"
        )
        self.logger_status.setWordWrap(True)
        research_layout.addWidget(self.logger_status)

        research_group.setLayout(research_layout)
        root.addWidget(research_group)

        # ====================================================
        # COMMAND RESULT
        # ====================================================

        result_group = QGroupBox(
            "VOICE RECOGNITION RESULT"
        )

        result_layout = QVBoxLayout()

        self.transcript_label = QLabel(
            "Transcript: -"
        )

        self.command_label = QLabel(
            "Command: NONE"
        )

        self.confidence_label = QLabel(
            "Command confidence: 0.00"
        )

        result_layout.addWidget(
            self.transcript_label
        )

        result_layout.addWidget(
            self.command_label
        )

        result_layout.addWidget(
            self.confidence_label
        )

        result_group.setLayout(
            result_layout
        )

        root.addWidget(
            result_group
        )

        # ====================================================
        # LOG
        # ====================================================

        log_group = QGroupBox(
            "SYSTEM LOG"
        )

        log_layout = QVBoxLayout()

        self.log_text = QTextEdit()

        self.log_text.setReadOnly(
            True
        )

        self.log_text.setMaximumHeight(
            100
        )

        log_layout.addWidget(
            self.log_text
        )

        log_group.setLayout(
            log_layout
        )

        root.addWidget(
            log_group
        )

        # ====================================================
        # STYLE
        # ====================================================

        self.setStyleSheet(
            """
            QGroupBox {
                font-weight: bold;
                margin-top: 10px;
                padding-top: 10px;
            }

            QPushButton {
                min-height: 36px;
                font-size: 13px;
                font-weight: bold;
            }

            QLabel {
                font-size: 14px;
            }
            """
        )

        self.log(
            "HRI GUI started"
        )

        self.log(
            f"Robot IP = {ROBOT_IP}"
        )

        self.log(
            f"Video = UDP UNICAST port {VIDEO_PORT}"
        )

        self.log(
            f"Voice command topic = "
            f"{VOICE_COMMAND_TOPIC}"
        )

        self.log(
            f"Robot control mode topic = "
            f"{CONTROL_MODE_TOPIC}"
        )

    # --------------------------------------------------------
    # MODE
    # --------------------------------------------------------

    def set_robot_control_mode(self, mode):
        try:
            self.ros_node.publish_control_mode(mode)
            self.log(f"[ROBOT] Control mode = {mode}")
        except Exception as exc:
            self.log(f"[ROBOT] Control mode error: {exc}")

    def select_mode(self, mode):

        self.current_mode = mode

        self.update_mode_buttons()

        self.log(
            f"HRI mode selected: {mode}"
        )

        if mode == "ONLY VOICE":

            # Mode 2 = VOICE on Robot Joy Relay
            self.set_robot_control_mode(2)

            if self.voice_thread is not None and self.voice_thread.model is not None:
                self.voice_status.setText("Voice: READY")
                self.btn_voice_start.setEnabled(True)
            else:
                self.voice_status.setText("Voice: Loading Whisper...")
                self.btn_voice_start.setEnabled(False)

        elif mode == "ONLY GESTURE":

            self.voice_status.setText(
                "Voice: disabled in GESTURE mode"
            )

            self.btn_voice_start.setEnabled(
                False
            )

            QMessageBox.information(
                self,
                "Gesture Mode",
                "ONLY GESTURE ยังไม่ได้ implement "
                "ในไฟล์เวอร์ชันแรก"
            )

        elif mode == "VOICE + GESTURE / FUSION":

            self.voice_status.setText(
                "Voice: waiting for Fusion implementation"
            )

            self.btn_voice_start.setEnabled(
                False
            )

            QMessageBox.information(
                self,
                "Fusion Mode",
                "VOICE + GESTURE / FUSION "
                "จะ implement ในไฟล์ถัดไป"
            )

    def update_mode_buttons(self):

        normal = (
            "background-color: #E6E6E6;"
        )

        selected = (
            "background-color: #90EE90;"
            "font-weight: bold;"
        )

        self.btn_voice_mode.setStyleSheet(
            selected
            if self.current_mode == "ONLY VOICE"
            else normal
        )

        self.btn_gesture_mode.setStyleSheet(
            selected
            if self.current_mode == "ONLY GESTURE"
            else normal
        )

        self.btn_fusion_mode.setStyleSheet(
            selected
            if self.current_mode ==
            "VOICE + GESTURE / FUSION"
            else normal
        )

    # --------------------------------------------------------
    # VIDEO
    # --------------------------------------------------------

    def start_video(self):

        self.video_thread = VideoThread()

        self.video_thread.frame_received.connect(
            self.update_video
        )

        self.video_thread.status_changed.connect(
            self.update_video_status
        )

        self.video_thread.start()

    def update_video_status(self, text):

        self.video_status.setText(
            text
        )

        self.log(
            "[VIDEO] " + text
        )

    def update_video(self, frame):

        if frame is None:
            return

        rgb = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        h, w, ch = rgb.shape

        bytes_per_line = (
            ch * w
        )

        image = QImage(
            rgb.data,
            w,
            h,
            bytes_per_line,
            QImage.Format_RGB888
        )

        pixmap = QPixmap.fromImage(
            image
        )

        pixmap = pixmap.scaled(
            self.video_label.width(),
            self.video_label.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )

        self.video_label.setPixmap(
            pixmap
        )

    # --------------------------------------------------------
    # VOICE
    # --------------------------------------------------------

    def ensure_voice_thread(self):

        if (
            self.voice_thread is not None
            and self.voice_thread.isRunning()
        ):
            return True

        self.voice_thread = VoiceThread()

        self.voice_thread.status_changed.connect(
            self.update_voice_status
        )

        self.voice_thread.ready_signal.connect(
            self.on_voice_ready
        )

        self.voice_thread.transcript_ready.connect(
            self.on_transcript
        )

        self.voice_thread.error_signal.connect(
            self.on_voice_error
        )

        self.voice_thread.start()

        # The model loading occurs in the thread.
        return True

    def on_voice_ready(self, ready):

        if ready and self.current_mode == "ONLY VOICE":
            self.voice_status.setText("Voice: READY")
            self.btn_voice_start.setEnabled(True)
            self.log("[VOICE] Whisper model ready - microphone enabled")
        elif not ready:
            self.btn_voice_start.setEnabled(False)

    def start_voice(self):

        if self.current_mode != "ONLY VOICE":
            return

        self.ensure_voice_thread()

        if self.voice_thread.model is None:
            self.voice_status.setText("Voice: WAITING FOR WHISPER...")
            return

        # R3 is Thai-only.
        self.voice_thread.language = "th"

        self.btn_voice_start.setText(
            "🔴 LISTENING... RELEASE TO PROCESS"
        )

        self.btn_voice_start.setStyleSheet(
            "background-color: #FFAAAA;"
        )

        self.voice_status.setText(
            "Voice: LISTENING..."
        )

        self.log(
            "[VOICE] Microphone recording started"
        )

        # Start experiment timing with the recording request.
        self._experiment_start_time = time.monotonic()

        self.voice_thread.start_recording()

    def stop_voice(self):

        if self.voice_thread is None:
            return

        self.voice_thread.stop_recording()

        self.btn_voice_start.setText(
            "🎤 PRESS AND HOLD TO SPEAK"
        )

        self.btn_voice_start.setStyleSheet(
            ""
        )

        self.log(
            "[VOICE] Microphone recording stopped"
        )

    def update_voice_status(self, text):

        self.voice_status.setText(
            text
        )

        self.log(
            "[VOICE] " + text
        )

    def on_transcript(
        self,
        transcript,
        command,
        confidence
    ):

        self.last_transcript = transcript
        self.last_command = command

        self.transcript_label.setText(
            f"Transcript: {transcript}"
        )

        self.command_label.setText(
            f"Command: {command}"
        )

        self.confidence_label.setText(
            f"Command confidence: "
            f"{confidence:.2f}"
        )

        self.log(
            f"[VOICE] Transcript = {transcript}"
        )

        self.log(
            f"[VOICE] Command = {command}, "
            f"confidence = {confidence:.2f}"
        )

        # ----------------------------------------------------
        # Research experiment logging
        # ----------------------------------------------------
        expected = self.expected_command_combo.currentText()
        if expected == "NOT SET":
            expected = ""
        elif expected == "NONE / NON-COMMAND":
            expected = "NONE"

        noise_condition = self.noise_combo.currentText()
        if noise_condition == "NOT SET":
            noise_condition = ""

        selected_language = "Thai"

        if expected:
            if expected == "NONE":
                # NONE / NON-COMMAND includes trials where the system
                # correctly detects that no speech was present.
                result = (
                    "CORRECT"
                    if command in ("UNKNOWN", "NO_SPEECH")
                    else "INCORRECT"
                )
            else:
                # A no-speech trial is an unsuccessful recognition
                # for a commanded motion.
                result = (
                    "CORRECT"
                    if command == expected
                    else "INCORRECT"
                )
        else:
            result = "UNLABELED"

        latency = 0.0
        if hasattr(self, "_experiment_start_time"):
            latency = time.monotonic() - self._experiment_start_time

        try:
            trial_id = self.experiment_logger.log_trial(
                mode=self.current_mode,
                language=selected_language,
                transcript=transcript,
                expected_command=expected,
                predicted_command=command,
                confidence=confidence,
                result=result,
                recording_duration_s=0.0,
                asr_latency_s=0.0,
                classification_latency_s=0.0,
                total_latency_s=latency,
                noise_condition=noise_condition,
                notes="GUI auto-log",
            )
            self.logger_status.setText(
                f"CSV: {self.experiment_logger.get_path()} | "
                f"Last trial: {trial_id} | {result}"
            )
            self.log(f"[RESEARCH] Trial {trial_id} -> {result}")
        except Exception as exc:
            self.log(f"[RESEARCH ERROR] CSV logging failed: {exc}")


        # Only actual robot commands are allowed onto /voice_command.
        # UNKNOWN and NO_SPEECH are research outcomes, not robot commands.
        if command in COMMAND_KEYWORDS:

            self.send_robot_command(
                command
            )

    def send_robot_command(self, command):

        if not command:
            return

        # ----------------------------------------------------
        # Current HRI interface:
        #
        # Notebook ROS2
        #       |
        #       v
        # /voice_command (std_msgs/String)
        #       |
        #       v
        # Robot ROS2 node
        #
        # This intentionally does NOT invent a motor-control
        # topic. The robot-side node can translate:
        #
        # FORWARD  -> robot motion
        # BACKWARD -> robot motion
        # LEFT     -> robot motion
        # RIGHT    -> robot motion
        # STOP     -> robot motion
        # ----------------------------------------------------

        try:

            self.ros_node.publish_command(
                command
            )

            self.log(
                f"[ROBOT] Sent command: {command}"
            )

        except Exception as exc:

            self.log(
                f"[ROBOT] Command error: {exc}"
            )

    # --------------------------------------------------------
    # ERROR
    # --------------------------------------------------------

    def on_voice_error(self, message):

        self.voice_status.setText(
            "Voice: ERROR"
        )

        self.log(
            "[VOICE ERROR] " + message
        )

        QMessageBox.warning(
            self,
            "Voice System Error",
            message
        )

    # --------------------------------------------------------
    # LOG
    # --------------------------------------------------------

    def log(self, message):

        timestamp = time.strftime(
            "%H:%M:%S"
        )

        self.log_text.append(
            f"[{timestamp}] {message}"
        )

        scrollbar = (
            self.log_text.verticalScrollBar()
        )

        scrollbar.setValue(
            scrollbar.maximum()
        )

        print(
            f"[{timestamp}] {message}"
        )

    # --------------------------------------------------------
    # CLOSE
    # --------------------------------------------------------

    def closeEvent(self, event):

        self.log(
            "Shutting down HRI GUI..."
        )

        # Stop voice.
        if self.voice_thread is not None:

            self.voice_thread.stop()

            if self.voice_thread.isRunning():

                self.voice_thread.wait(
                    2000
                )

        # Stop video.
        if self.video_thread is not None:

            self.video_thread.stop()

            if self.video_thread.isRunning():

                self.video_thread.wait(
                    2000
                )

        event.accept()


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 60)
    print("SCOUT ROBOT HRI GUI")
    print("Version: ONLY VOICE + B1 R2.1 NO-SPEECH RECOVERY + STOP ASR RESCUE + RESEARCH LOGGER")
    print("=" * 60)

    print(
        f"ROBOT_IP      = {ROBOT_IP}"
    )

    print(
        f"VIDEO_PORT    = {VIDEO_PORT}"
    )

    print(
        f"AUDIO_RX_PORT = {AUDIO_RX_PORT}"
    )

    print(
        f"AUDIO_TX_PORT = {AUDIO_TX_PORT}"
    )

    print(
        f"VOICE_TOPIC   = {VOICE_COMMAND_TOPIC}"
    )

    # --------------------------------------------------------
    # ROS 2
    # --------------------------------------------------------

    rclpy.init()

    ros_node = HRICommandNode()

    # --------------------------------------------------------
    # Qt
    # --------------------------------------------------------

    app = QApplication(
        sys.argv
    )

    window = HRIGUI(
        ros_node
    )

    window.show()

    # --------------------------------------------------------
    # ROS spin
    # --------------------------------------------------------

    ros_thread = ROS2Thread(
        ros_node
    )

    ros_thread.start()

    try:

        exit_code = app.exec_()

    finally:

        ros_thread.stop()

        if ros_thread.isRunning():

            ros_thread.wait(
                1000
            )

        try:
            ros_node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(
        exit_code
    )


if __name__ == "__main__":
    main()

import csv
import math
import os
import re
import socket
import statistics
import struct
import time

PATTERN = re.compile(r"0x[0-9a-fA-F]+:\s*((?:[0-9a-fA-F]{8}\s*)+)")


def f32(word):
    return struct.unpack("<f", struct.pack("<I", word & 0xFFFFFFFF))[0]


def u16lo(word):
    return word & 0xFFFF


def u16hi(word):
    return (word >> 16) & 0xFFFF


def s16(value):
    return value - 65536 if value & 0x8000 else value


def clean_telnet_bytes(data):
    return bytes(
        value if (value in (9, 10, 13) or 32 <= value < 127) else 32
        for value in data
    )


def parse_words(text):
    words = []
    for match in PATTERN.finditer(text):
        words.extend(int(value, 16) for value in match.group(1).split())
    return words


def recv_prompt(sock, timeout=0.18):
    deadline = time.monotonic() + timeout
    chunks = []
    while time.monotonic() < deadline:
        try:
            data = sock.recv(65536)
            if data:
                chunks.append(data)
                joined = b"".join(chunks)
                if b"> " in joined or joined.rstrip().endswith(b">"):
                    break
        except Exception:
            time.sleep(0.001)
    return clean_telnet_bytes(b"".join(chunks)).decode("ascii", "ignore")


def openocd_cmd(sock, command):
    sock.sendall((command + "\r\n").encode("ascii"))
    return recv_prompt(sock)


def corr(a_values, b_values):
    if len(a_values) < 3:
        return float("nan")
    mean_a = statistics.fmean(a_values)
    mean_b = statistics.fmean(b_values)
    var_a = sum((value - mean_a) ** 2 for value in a_values)
    var_b = sum((value - mean_b) ** 2 for value in b_values)
    if var_a <= 1.0e-12 or var_b <= 1.0e-12:
        return float("nan")
    cov = sum(
        (a_value - mean_a) * (b_value - mean_b)
        for a_value, b_value in zip(a_values, b_values)
    )
    return cov / math.sqrt(var_a * var_b)


def best_scale(x_values, y_values):
    denom = sum(value * value for value in x_values)
    if denom <= 1.0e-12:
        return float("nan")
    return sum(x_value * y_value for x_value, y_value in zip(x_values, y_values)) / denom


def rms(values):
    return math.sqrt(statistics.fmean([value * value for value in values])) if values else 0.0


def main():
    duration_s = float(os.environ.get("PENDULUM_DEBUG_DURATION_S", "30.0"))
    sock = socket.create_connection(("127.0.0.1", 4444), timeout=2)
    sock.settimeout(0.015)
    recv_prompt(sock, 0.3)

    rows = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        elapsed = time.monotonic() - t0
        flow = parse_words(openocd_cmd(sock, "mdw 0x20016328 54"))
        ekf = parse_words(openocd_cmd(sock, "mdw 0x20016818 20"))
        vofa = parse_words(openocd_cmd(sock, "mdw 0x20013e50 34"))
        debug = parse_words(openocd_cmd(sock, "mdw 0x20014f04 46"))
        imu = parse_words(openocd_cmd(sock, "mdw 0x200165f8 8"))

        if len(flow) < 54 or len(vofa) < 34 or len(debug) < 46 or len(imu) < 8:
            continue

        h_m = f32(flow[3])
        raw_x = s16(u16lo(flow[41]))
        raw_y = s16(u16hi(flow[41]))
        flow_x_no_height = raw_x * 0.01
        flow_y_no_height = raw_y * 0.01
        flow_x_with_height = flow_x_no_height * h_m
        flow_y_with_height = flow_y_no_height * h_m

        roll_acc = f32(imu[0])
        pitch_acc = f32(imu[1])
        roll_gyro = f32(imu[2])
        pitch_gyro = f32(imu[3])
        alpha = f32(imu[6])
        roll_est = alpha * roll_gyro + (1.0 - alpha) * roll_acc
        pitch_est = alpha * pitch_gyro + (1.0 - alpha) * pitch_acc

        row = {
            "t_s": elapsed,
            "sample_ms": flow[13],
            "height_m": h_m,
            "raw_flow_x": raw_x,
            "raw_flow_y": raw_y,
            "flow_x_no_height_m_s": flow_x_no_height,
            "flow_y_no_height_m_s": flow_y_no_height,
            "flow_x_with_height_m_s": flow_x_with_height,
            "flow_y_with_height_m_s": flow_y_with_height,
            "app_vx_m_s": f32(flow[10]),
            "app_vy_m_s": f32(flow[11]),
            "vofa_vx_m_s": f32(vofa[3]),
            "vofa_vy_m_s": f32(vofa[4]),
            "acc_nav_x_m_s2": f32(vofa[0]),
            "acc_nav_y_m_s2": f32(vofa[1]),
            "acc_nav_z_m_s2": f32(vofa[2]),
            "rate_roll_rad_s": f32(debug[36]),
            "rate_pitch_rad_s": f32(debug[37]),
            "rate_roll_dps": f32(debug[36]) * 57.2957795,
            "rate_pitch_dps": f32(debug[37]) * 57.2957795,
            "moment_roll_n_m": f32(debug[39]),
            "moment_pitch_n_m": f32(debug[40]),
            "tilt_out_0_rad": f32(debug[18]),
            "tilt_out_1_rad": f32(debug[19]),
            "force_cmd_x_n": f32(debug[9]),
            "force_cmd_y_n": f32(debug[10]),
            "horizontal_scale": f32(debug[42]),
            "protection_flags": debug[45],
            "roll_est_deg": roll_est,
            "pitch_est_deg": pitch_est,
            "roll_acc_deg": roll_acc,
            "pitch_acc_deg": pitch_acc,
            "alpha": alpha,
            "dt_ms": f32(imu[7]),
            "vel_ref_x_m_s": f32(vofa[6]),
            "vel_ref_y_m_s": f32(vofa[7]),
            "vel_loop_active": f32(vofa[30]),
            "flow_quality": flow[42] & 0xFF,
            "flow_valid": flow[12] & 0xFF,
        }
        if len(ekf) >= 20:
            row["ekf_vx_m_s"] = f32(ekf[0])
            row["ekf_vy_m_s"] = f32(ekf[1])
            row["ekf_updates"] = ekf[9]
            row["ekf_rejects"] = ekf[10]
            row["ekf_skips"] = ekf[11]
        rows.append(row)

    sock.sendall(b"exit\r\n")
    sock.close()

    os.makedirs(".tmp", exist_ok=True)
    output_path = ".tmp/pendulum_openocd_velocity_debug.csv"
    if rows:
        fieldnames = list(rows[0].keys())
        with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    print(f"samples={len(rows)} duration_s={time.monotonic() - t0:.2f} csv={output_path}")
    if not rows:
        return

    for name in [
        "height_m",
        "raw_flow_x",
        "raw_flow_y",
        "flow_x_no_height_m_s",
        "flow_y_no_height_m_s",
        "flow_x_with_height_m_s",
        "flow_y_with_height_m_s",
        "app_vx_m_s",
        "app_vy_m_s",
        "vofa_vx_m_s",
        "vofa_vy_m_s",
        "rate_roll_dps",
        "rate_pitch_dps",
        "moment_roll_n_m",
        "moment_pitch_n_m",
        "tilt_out_0_rad",
        "tilt_out_1_rad",
        "roll_est_deg",
        "pitch_est_deg",
        "vel_loop_active",
    ]:
        values = [row[name] for row in rows]
        print(
            f"{name}: min={min(values): .4f} max={max(values): .4f} "
            f"mean={statistics.fmean(values): .4f} rms={rms(values): .4f}"
        )

    motion_rows = [
        row for row in rows
        if abs(row["raw_flow_x"]) + abs(row["raw_flow_y"]) >= 3
    ]
    print(f"motion_samples={len(motion_rows)}")

    if len(motion_rows) >= 10:
        comparisons = [
            ("x", "flow_x_no_height_m_s", "flow_x_with_height_m_s", "rate_pitch_rad_s"),
            ("y", "flow_y_no_height_m_s", "flow_y_with_height_m_s", "rate_roll_rad_s"),
        ]
        for axis, no_height_key, with_height_key, rate_key in comparisons:
            no_height = [row[no_height_key] for row in motion_rows]
            with_height = [row[with_height_key] for row in motion_rows]
            omega = [row[rate_key] for row in motion_rows]
            radius_no_height = best_scale(omega, no_height)
            radius_with_height = best_scale(omega, with_height)
            print(
                f"axis_{axis}: corr(no_height,omega)={corr(no_height, omega): .3f} "
                f"R_fit_no_height={radius_no_height: .3f}m | "
                f"corr(with_height,omega)={corr(with_height, omega): .3f} "
                f"R_fit_with_height={radius_with_height: .3f}m"
            )

        err_with_height = max(
            max(
                abs(row["app_vx_m_s"] - row["flow_x_with_height_m_s"]),
                abs(row["app_vy_m_s"] - row["flow_y_with_height_m_s"]),
            )
            for row in rows
        )
        err_no_height = max(
            max(
                abs(row["app_vx_m_s"] - row["flow_x_no_height_m_s"]),
                abs(row["app_vy_m_s"] - row["flow_y_no_height_m_s"]),
            )
            for row in rows
        )
        print(
            f"app_matches_with_height_maxerr={err_with_height:.6f}; "
            f"app_matches_no_height_maxerr={err_no_height:.6f}"
        )

        top_rows = sorted(
            motion_rows,
            key=lambda row: abs(row["flow_x_no_height_m_s"]) + abs(row["flow_y_no_height_m_s"]),
            reverse=True,
        )[:12]
        print(
            "top: t,h,rawx,rawy,noH_x,noH_y,withH_x,withH_y,"
            "vofa_x,vofa_y,rate_roll_dps,rate_pitch_dps,"
            "moment_roll,moment_pitch,roll,pitch"
        )
        for row in top_rows:
            print(
                f"{row['t_s']:5.2f},{row['height_m']:.3f},"
                f"{row['raw_flow_x']:5d},{row['raw_flow_y']:5d},"
                f"{row['flow_x_no_height_m_s']: .3f},"
                f"{row['flow_y_no_height_m_s']: .3f},"
                f"{row['flow_x_with_height_m_s']: .3f},"
                f"{row['flow_y_with_height_m_s']: .3f},"
                f"{row['vofa_vx_m_s']: .3f},{row['vofa_vy_m_s']: .3f},"
                f"{row['rate_roll_dps']: .1f},{row['rate_pitch_dps']: .1f},"
                f"{row['moment_roll_n_m']: .3f},{row['moment_pitch_n_m']: .3f},"
                f"{row['roll_est_deg']: .1f},{row['pitch_est_deg']: .1f}"
            )


if __name__ == "__main__":
    main()

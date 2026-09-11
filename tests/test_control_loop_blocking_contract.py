"""S6 control contracts."""

from __future__ import annotations

import hashlib
from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
SERVO_CAL = ROOT / "App" / "Src" / "app_servo_cal.c"
SERVO_CAL_HEADER = ROOT / "App" / "Inc" / "app_servo_cal.h"
CONTROL = ROOT / "App" / "Src" / "app_control.c"
CONTROL_CORE = ROOT / "App" / "Src" / "app_control_core.c"
SERVOCAL_CMD = ROOT / "App" / "Src" / "app_cmd_servocal.c"
IMUCAL_CMD = ROOT / "App" / "Src" / "app_cmd_imucal.c"
RCMAP_CMD = ROOT / "App" / "Src" / "app_cmd_rcmap.c"
FLOW_CMD = ROOT / "App" / "Src" / "app_cmd_flow.c"
SYSTEM_CMD = ROOT / "App" / "Src" / "app_cmd_system.c"
DIAG_CMD = ROOT / "App" / "Src" / "app_cmd_diag.c"
CONTROL_INTERNAL = ROOT / "App" / "Inc" / "app_control_internal.h"
CMAKE = ROOT / "CMakeLists.txt"

STEP_A_BODY_SHA256 = {
    # 2026-09-11: structured text is now also mirrored to the on-board
    # Bluetooth UART while that link is in use, so BT sees READY/heartbeat
    # lines like USB does. Guarded by APP_MaintUART_IsLinkActive() so an
    # idle BT port does not block the UART task on every line.
    "app_control_queue_proto_text": "882c3d21b3c08fd35aa6491916e1426e78c8b7384e68511b33727048c9f8af06",
    "app_control_tokenize": "5186a0c9ef1a832f2619926b25b3e14cf2c4fe859baa2075cc28e0bff9f12923",
    "app_control_parse_u32": "9b35592f3df9183e87294ba31661553305887e7be944392ff724be2a570820d7",
    "app_control_parse_i32": "6c8d3ff6ef993cc6bd8726e99676aa1cbea12a2e4267c74a0b205fb1d98f1923",
    "app_control_crc32_update": "fca3b9139ae7aab310b73813e60e704bb163d463413a953cd4cb83f9c4ecf6c3",
    "app_control_crc32": "3169f7344b5ead9306fc097785bd32626479e1b7c01040d8b8d6c44b12d2998a",
    "app_control_token_value": "9305c3293c9348edbcd2f09908a04a8d54270fc4441eee05a146d43167ff968e",
}
STEP_B_LEGACY_BODY_SHA256 = {
    # C extracts only the contiguous SERVOCAL persisted fragment; the rest of
    # sync is pinned by this direct-parent hash and the fragment below.
    # S7 adds one persisted-servo-type observer beside the existing SERVOCAL
    # observer; all earlier sync semantics remain pinned by the new full body.
    "app_control_imuframe_sync_param": "ab1cb18e8324e2b0fced1ebcbc74bbf9ad772078ff0375efa666e7cbbc254e66",
    "app_control_imucal_safety": "87327cf50ef57f4566ad276d868912f8bbac6f3e99524e80b20b2b0b544d4c78",
}
STEP_B_ACCESSOR_BODIES = {
    "const void *app_control_internal_imucal_confirmed_record(void)": (
        "{\n    return &control_imucal_confirmed;\n}"
    ),
    "uint32_t app_control_internal_imucal_confirmed_generation(void)": (
        "{\n    return control_imucal_confirmed_generation;\n}"
    ),
    "uint8_t app_control_internal_imucal_confirmed_valid(void)": (
        "{\n    return control_imucal_confirmed_valid;\n}"
    ),
    "void app_control_internal_imuframe_sync_param(void)": (
        "{\n    app_control_imuframe_sync_param();\n}"
    ),
    "const char *app_control_internal_imucal_safety(": (
        "{\n"
        "    return app_control_imucal_safety(\n"
        "        (StabilizerValidationImuSnapshot *)snapshot,\n"
        "        require_sequence_progress);\n"
        "}"
    ),
    "uint8_t app_control_internal_imucal_upload_state(void)": (
        "{\n    return (uint8_t)control_imucal_upload.state;\n}"
    ),
    "uint8_t app_control_internal_imucal_applied(void)": (
        "{\n    return control_imucal_applied;\n}"
    ),
    "uint8_t app_control_internal_imucal_commit_pending(void)": (
        "{\n    return control_imucal_commit_pending;\n}"
    ),
    "uint8_t app_control_internal_imuframe_confirmed_code(void)": (
        "{\n    return control_imuframe_confirmed_code;\n}"
    ),
}
STEP_C_BODY_SHA256 = {
    "app_control_servocal_clear_preview": "b1854d00dff1cdcfcf730b1c1e10b03aa2ef2d85f6bd37a55f3cde2a8505f84e",
    "app_control_servocal_set_event": "a4aae2e90420e2db7233774aaa71d786c3040f2360e1a22bfb9c2e65128b9dc7",
    "app_control_report_servocal_record": "41de7a2923ed57076d428b92cfa7dc33dbfcf0f1e42329a8262188aba60e1283",
    "app_control_report_servocal": "47dff691d685afff45143df773e0eb692c1ae47b5c90af6743965299cf2b15d4",
    "app_control_parse_servocal": "8ac31c2ea6d80c4f2c0c6c0ac730b903d738ae22ff746d2fa500dd226e15cb5b",
    "app_control_handle_servocal": "75f9f7389664a52523366bd3d3b6390d5b5905d90e38282214d752ccc9aab07d",
    "app_control_service_servocal": "ec23df6b66699a1aa7aeec88247c39007acbe3d5dbcb46d81c1a994b4f76d135",
}
STEP_C_STATE_NAMES = {
    "control_servocal_preview",
    "control_servocal_pending_record",
    "control_servocal_preview_generation",
    "control_servocal_last_request",
    "control_servocal_applied",
    "control_servocal_commit_pending",
    "control_servocal_last_event",
    "control_servocal_last_reason",
}
STEP_C_PERSISTED_FRAGMENT = """    if (control_servocal_commit_pending != 0U) {
        if (memcmp(&calibration,
                   &control_servocal_pending_record,
                   sizeof(calibration)) == 0) {
            app_control_servocal_set_event("committed", "none");
        } else {
            app_control_servocal_set_event("commit_failed", "record_mismatch");
        }
        app_control_servocal_clear_preview();
    } else if (control_servocal_applied != 0U) {
        app_control_servocal_set_event("reverted", "persisted_changed");
        app_control_servocal_clear_preview();
    }"""
STEP_C_INIT_FRAGMENT = """    memset(&control_servocal_preview, 0,
           sizeof(control_servocal_preview));
    memset(&control_servocal_pending_record, 0,
           sizeof(control_servocal_pending_record));
    control_servocal_preview_generation = 0U;
    control_servocal_last_request = 0U;
    control_servocal_applied = 0U;
    control_servocal_commit_pending = 0U;
    app_control_servocal_set_event("init", "none");
    APP_Stabilizer_SetServoCalibrationCandidateArmLock(0U);"""
STEP_D1_BODY_SHA256 = {
    "app_control_imucal_clear_candidate": "e6c3bd116af8d32f7df1af6d73c4b4dc9784ff9aa908a13728f70830e1998cec",
    "app_control_imucal_set_event": "48bffc9d5f42d37dedd67eadc50e3be522390e597996c655e43be709d4279722",
    "app_control_report_imucal": "a303d165eafbc51114194e2fa9f991f8dcc4a9c39d420aeaffe4fe8f321b3d0e",
    "app_control_parse_hex_u32": "4f907fca5afa2296619f2e43fdfe4495d2b2b8fc24931c3a2030c1f4833811c5",
    "app_control_imucal_candidate_context_error": "5f851c8c07572645177909f463d785c40329083c19854d506460d5423fe8fd34",
    "app_control_imucal_transfer_result": "900f5ec2f618107ad65f53ae0c53c23c66528ab9fbfebd31eb1edb2f214a9c85",
    "app_control_handle_imucal": "23534947668ae744ef42e2bb6b0986fafa9d9b8ada1e5fbc5e2500141790424a",
    "app_control_service_imucal": "5362e08b66811810813edb109e187f7378039ff8bdb9cc5579e67bfca68ead9c",
}
STEP_D1_STATE_NAMES = {
    "control_imucal_upload",
    "control_imucal_preview",
    "control_imucal_pending_record",
    "control_imucal_preview_generation",
    "control_imucal_last_request",
    "control_imucal_applied",
    "control_imucal_commit_pending",
    "control_imucal_last_event",
    "control_imucal_last_reason",
}
STEP_D1_LEGACY_BODY_SHA256 = {
    # S7 adds only servo-type init/sync hooks to these legacy owners.
    "APP_Control_Init": "29b4da5acc7963cd71b0ce1ca0b3c677ad41b8b4e3f0ac80a9ee1e9a1b01b8f2",
    "app_control_imuframe_sync_param": "ab1cb18e8324e2b0fced1ebcbc74bbf9ad772078ff0375efa666e7cbbc254e66",
    "app_control_imucal_safety": "87327cf50ef57f4566ad276d868912f8bbac6f3e99524e80b20b2b0b544d4c78",
    "app_control_handle_acceptance": "f10cf7512f9fab6a1e9ccfdc1957e8b2a3d2b5d4803c50b74dc27e3c2627e9a9",
}
STEP_D2_PARENT_COMMIT = "ebc106f6fa8c2b16d7e7d5dda7fe8f43d1b878dd"
STEP_D2_BODY_SHA256 = {
    "app_control_apply_rc_config": "3ad6a64240584cc332457b241b7df41c90523f90da956ec5b3ecba52f77b3f8b",
    "app_control_report_rc_map": "06f31a4cae822bd0ef7f38ac839d4af72fe39d6c078f40ee8584321fe2ba4c05",
    "app_control_report_rc_live": "944a9a81b307258c01c44ef2a510d083bd8a4ffdc7fc14407c885c90a118bccd",
    "app_control_rc_write_allowed": "91115b13961e4c5835ce97950fde25a4e3845fa041e2bc10445171fdc8fbbaee",
}
STEP_D2_PARENT_HANDLER_SHA256 = (
    "6dbe4c5ae3411cbf0a9c72239a0413b5fbd184d2f24af0ce1cdeac8a44e11920"
)
STEP_D2_CHILD_HANDLER_SHA256 = (
    "6a5dfe240a7011112c5171ad5060b584a50e30c265d5bb7a98eebb9b0c3d4676"
)
STEP_D2_DELEGATED_PERSIST_FRAGMENT = """        save_status = app_control_internal_commit_config_persist();
        if (save_status != APP_FLASH_SERVICE_OK) {
            app_control_report_rc_map("commit_failed");
            return;
        }"""
STEP_D2_PARENT_PERSIST_FRAGMENT = """        save_status = app_control_save_config();
        if (save_status != APP_FLASH_SERVICE_OK) {
            control_config.last_flash_status = (uint8_t)save_status;
            app_control_report_rc_map("commit_failed");
            return;
        }
        control_config.last_flash_status = (uint8_t)save_status;
        control_config.loaded_from_flash = 1U;
        control_config.flash_valid = 1U;"""
STEP_D2_PERSIST_HELPER_BODY = """{
    APP_FlashService_Status save_status;

    save_status = app_control_save_config();
    if (save_status != APP_FLASH_SERVICE_OK) {
        control_config.last_flash_status = (uint8_t)save_status;
        return (uint8_t)save_status;
    }
    control_config.last_flash_status = (uint8_t)save_status;
    control_config.loaded_from_flash = 1U;
    control_config.flash_valid = 1U;
    return (uint8_t)save_status;
}"""
STEP_D3_PARENT_COMMIT = "aa538a1cbb0e68b8e232c42a2b7096528ba2547a"
STEP_D3_BODY_SHA256 = {
    "app_control_parse_hex_byte": "822f7d5255be98c3cf3ef75a9fbbe59c94cfa0bf4e8a7b1105e78a0d2af318d4",
    # R-M5-5：新增 FLOW ZERO 子命令（清里程计），是本次授权的功能改动而非 D3 搬家
    # 走样，故重钉。其余两个函数体仍是 D3 父提交原样。
    "app_control_handle_flow": "697d5ad0f1bbbb7091eaed46188a264be692720004a3d5ec10a22cf59f6136a9",
    # 符号归一化（2026-09-06）：`FLOW comp` 的 source= 字段原样报的是
    # controller_legacy_x_forward_y_right，而光流现在在传感器出口就已经转成
    # 规范 FLU，那个标签会把上位机和后续实录一起带偏。这是被授权的口径修正，
    # 不是 D3 搬家走样，故重钉。函数结构未变。
    "app_control_report_flow": "8a4dc201743f61e3e24b3765ea8a64e9d598175dff01094ecc6b129ca5a6c706",
}
STEP_D3_ACCEPTANCE_BODY_SHA256 = (
    "55bdb96ca6630a2e7036c0e4932ec33822ff82f8e54f455242fe7a2df5f9fdb2"
)
STEP_D3_ACCESSOR_BODY = """{
    return app_control_acceptance_milli(value);
}"""
STEP_D4_PARENT_COMMIT = "e44770bbaebd836d0df808bee80961db7db5cda0"
STEP_D4_SYSTEM_BODY_SHA256 = {
    # S7 advertises the newly registered SERVOTYPE? command.
    # 2026-09-11：能力表新增 IMUSEL（STATUS/BUS/RAW）与通用探针 MEM/SPI/I2C/UART。
    # 后者是授权新增的调试出口，动机见 App/Src/app_cmd_probe.c 文件头。首刷 MicoAir743v2 时
    # BMI088 没上岗而 BMI270 顶上，现有诊断一个字都说不出原因——选型记账早就存在
    # 于 SVC_IMU_Selection，只是没有出口。这是被授权的功能新增，不是 D4 搬家走样。
    "app_control_report_caps": "455ddba198796f88c260faeb09a51babee4c19cb6dd81b55e926c164fb2fe5c1",
    # 2026-09-11：WiFi 诊断原样印死 `pin=PC6`，而 PC6 在 MicoAir743v2 上是
    # USART6_TX（ELRS 发送脚），Ai-WB2 使能脚已不存在。改为 pin=none，口径修正。
    "app_control_report_wifi": "0f79bf2ce5d31bcd9a87db2699ed26f434646331f458b025c0d2932fda74bba4",
    # 2026-09-11: RTOS? now also reports STABILIZER and TELEM, and every task
    # line carries eTaskGetState(). Diagnosing "stream=1 but not one byte"
    # took hours precisely because the telemetry task was neither listed nor
    # showed a scheduler state -- it was Ready and never scheduled.
    "app_control_report_task_stack": "c26ae4ec0d80322969af8dadfbfe578fbd7e5f5c3b5cef38b2e47874bae41b84",
    "app_control_report_rtos": "58f0bef3b98624f289b79a7214634dc6dd22c86b8cb6aa27f6885571bd90a380",
    # 2026-09-11：MODULES 之后跟发一条 ARM 状态行。MODULES 是上位机连上必发的
    # 那条，横幅因此第一次刷新就有内容，不用等自己的 2 Hz 轮询转到。
    # 报文体本身在 App/Src/app_cmd_arm.c，这里只是多了一个调用，不是 D4 搬家走样。
    "app_control_report_modules": "2f5b5ad4a5525a21b1734d7de6cd8b56da57da8e1c7b038b4bd6a74853b3b8f7",
    # 2026-09-11：STATUS? 紧跟 HW FLASH 之后多报一行 HW PARAMSTORE。上面那行说的是
    # 外部 SPI NOR（本板没有，恒 ok=0，是实话），单看它会被读成"配置保存坏了"；
    # 参数其实存在片内 Flash 上。多一条报文，函数结构未变。
    "app_control_report_status": "3f0f8e642b47e8390c489618fe0594ccf603b5b0922c0942122fd19d6be29223",
    "app_control_report_usb_cdc_stats": "7cc8a758ff5d3e9c184f30338ef9955b6e0383e8a15329f32303ef42ea1f0f9c",
    "app_control_report_uart_stats": "a007f9a28d3078df665affa2edbd4095c1891700263f8d89b20959e7155c9627",
    "APP_Control_ReportUartStats": "aa55a393bc18a14388f8ecad6d721ebce5ac7b425ee0c23b8cd9d6d653415d35",
}
STEP_D4_DIAG_BODY_SHA256 = {
    "app_control_imu_stage_name": "43d16b390cb2b218d8ac36c812ad83b3e1e00fd4e5ab4675ecec64eeb3ecc2df",
    "app_control_flash_ok": "4d79f0b726b0c237b9c2b87af8238047e8211ad471862be308019892f4616e8b",
    "app_control_age_text": "7571ac5413aa09024f96a1ae4d8d3dd75365255f5f770246797ac5dadecf4f5e",
    "app_control_flash_stage": "365f505fc55be49645ccc429a8986780887c099b014dd71c7d608ee2a6a74215",
    "app_control_baro_ok": "4ea85895d0472403204645d4d1fc865620f9e1a62db279aba9d6699fdb036a1e",
    "app_control_baro_stage": "b91be3cbaf71c6e95691702030b4d72e2d183112b56042025b7fa9fbe01f6cf7",
    "app_control_aiwb2_state_name": "98c840dadf57ea0d5437b3eb437bc2ea6e5c62029fb6d57e410db78a8e55349e",
    "app_control_parse_u32_auto": "e7cf8135b868f444397cbd63e33f55621b2b82b6c585951e59af4059237db45c",
    "app_control_protocol_err": "2ed34ed98edfa7e4582e34caa851c146857606d43c5e746c172d05b31cfdbc1c",
    "app_control_req_spl06": "71a70258158e3c1215c87e5bf85a32de7e38c0d14cef203d44f962191f8ced2a",
    "app_control_req_icm42688": "4805915e544657f7d537d4fdea16dcb098b786620f8b9b0bf060bd83edbefe84",
    "app_control_req_m9n": "6768cfd5dfe511c1ef7cb3422461425b15a50c63832632778f48b4df9b181c58",
    "app_control_req_mag": "5c1993ac6286decfbc1c87fddcd66da1e0942c900c31d24ea43b651f62c02f01",
    # 2026-09-11：新增 IMUSEL、通用探针（MEM/SPI/I2C/UART）与 ARM 三个分发分支，
    # 处理体分别在 App/Src/app_cmd_imusel.c、app_cmd_probe.c、app_cmd_arm.c，
    # 此处只多三个分支；另有 WIFI 分支里 pin=PC6 → pin=none 的口径修正。
    # 均为授权改动，不是 D4 搬家走样。
    "app_control_handle_req": "793f1fddff7d7f0d79daaa38752ea9be3a133e4a43ade56690aaa3a0cdfffbbc",
}
STEP_D4_PROTECTED_LEGACY_BODY_SHA256 = {
    "app_control_report_config": "c9d30f2565e2de3830fcaca89e6f8d86f163431bb079893d44779bf414a75164",
    # 2026-09-11：删掉了函数体第一行的 `APP_Flash_RefreshStatus();`。
    #
    # 那次探测要拿 flashBusMutex，去问一颗本板**根本没有**的外部 SPI NOR。
    # 遥测画像量到：蓝牙上每敲一次 `FLASH?`，遥测任务就被饿 0.5~0.9 秒
    # （持锁者被优先级继承顶到遥测之上）。而 `APP_Flash_GetStatus` 本来就带
    # 一次性惰性探测，注释里写明"NOR 不会中途长出来，一次结论就够"——
    # 也就是说这一行本来就是多余的第二次探测。删后实测：首次 592 ms（惰性那次，
    # 不可避免），之后 26 ms。**只减不增**，符合 app_control.c 的硬约束。
    "app_control_report_flash": "f2777a529061d462f8722c8f6d172d4cb513563d4017818bc638aa0fc572182d",
    "app_control_report_baro": "e108b9fb652b46f7735c8129ed888d323167ee085b0e672e3d8fddce59723752",
    "app_control_report_imu": "683dfc8099042862a6835572e894508333373f0ca84b27acda1e7cd309d54cc3",
    "app_control_token_u32": "8e1c6bdd61e5fc1253e46d639c3f559291b5a0e88f69bab2b817b5adea834b07",
}

D4_STUB_HEADER = r"""
#ifndef D4_STUBS_H
#define D4_STUBS_H
#include <stddef.h>
#include <stdint.h>

#define APP_CONTROL_SERVO_COUNT 2U
typedef struct { uint8_t id; uint16_t pulse_us; uint16_t time_ms; uint8_t mode; uint8_t enabled; } APP_ControlServoConfig;
typedef struct { uint8_t loaded_from_flash; uint8_t flash_valid; uint8_t last_flash_status; APP_ControlServoConfig servo[2]; } APP_ControlConfig;
void APP_Control_QueueText(const char *, ...);

/*
 * 2026-09-11: STATUS? also reports HW PARAMSTORE, naming the medium the
 * parameters actually live on.  The HW FLASH line above describes the external
 * SPI NOR, which this board does not have (ok=0 forever) -- read alone it looks
 * like "saving is broken".  These stubs only let app_cmd_system.c compile in
 * D4's isolated environment; the real ones live in app_flash_service.c.
 */
#define APP_CONTROL_CFG_SLOT_A 0x003FE000UL
typedef int APP_FlashService_Backend;
APP_FlashService_Backend APP_FlashService_BackendFor(uint32_t address);
const char *APP_FlashService_BackendName(APP_FlashService_Backend backend);
uint8_t APP_FlashService_IsLogStorageReady(void);

typedef struct { int32_t probe_status; int32_t status1_status; int32_t read_status; uint8_t manufacturer_id; uint8_t memory_type; uint8_t capacity_id; uint8_t status1; } APP_Flash_Status;
typedef struct { int32_t init_status; int32_t split_status; int32_t txrx_status; uint8_t product_id; uint8_t split_id; uint8_t txrx_id; uint8_t bmp280_id; uint8_t cs_level; uint8_t miso_level; } APP_Baro_Status;
typedef struct {
    uint8_t initialized; uint8_t init_stage; int32_t last_status; int32_t last_error;
    uint8_t who_am_i; uint32_t sample_count; int16_t accel_x_mg; int16_t accel_y_mg;
    int16_t accel_z_mg; int32_t gyro_x_mdps; int32_t gyro_y_mdps; int32_t gyro_z_mdps;
    int16_t temperature_cdeg; uint8_t diag_valid; uint8_t diag_mode0_tokmas;
    uint8_t diag_mode0_msb; uint8_t diag_mode0_bit0; uint8_t diag_mode3_tokmas;
    uint8_t diag_mode3_msb; uint8_t diag_mode3_bit0; uint8_t diag_best_mode;
    uint8_t diag_best_header; uint8_t diag_burst_m0_b0_1; uint8_t diag_burst_m0_b0_2;
    uint8_t diag_burst_m0_b0_3; uint8_t diag_burst_m0_b0_4;
    uint8_t diag_burst_m3_tok_1; uint8_t diag_burst_m3_tok_2;
    uint8_t diag_burst_m3_tok_3; uint8_t diag_burst_m3_tok_4;
} APP_IMU_Status;
typedef struct { uint8_t initialized; int32_t init_status; uint32_t baud_rate; uint32_t bytes; uint32_t frames; uint8_t valid; uint32_t age_ms; int velocity_source; uint8_t velocity_valid; uint32_t checksum_errors; float height_m; float vx_m_s; float vy_m_s; } APP_OPTICAL_FLOW_Status;
typedef struct {
    uint8_t initialized; int32_t init_status; int32_t last_status; int type;
    uint8_t address; uint8_t who_am_i; uint32_t sample_count; int16_t raw_x;
    int16_t raw_y; int16_t raw_z; int32_t x_mgauss; int32_t y_mgauss;
    int32_t z_mgauss; uint8_t detected_ist8310; uint8_t detected_hmc5883;
    uint8_t detected_qmc5883; uint8_t hmc_id_a; uint8_t hmc_id_b; uint8_t hmc_id_c;
} APP_MAG_Status;
typedef struct {
    uint8_t initialized; int32_t init_status; uint8_t fix_type; uint8_t valid_fix;
    uint8_t num_sv; uint32_t last_rx_ms; uint32_t packets; uint32_t nav_pvt_packets;
    uint32_t nmea_sentences; uint32_t nmea_gga_sentences; uint32_t baud_rate;
    uint32_t bytes; uint32_t checksum_errors; uint32_t nmea_checksum_errors;
    uint32_t payload_overflows; uint32_t nmea_overflows; uint32_t rx_restarts;
    uint32_t uart_errors; uint32_t last_uart_error; uint32_t config_writes;
    int32_t lon_deg_e7; int32_t lat_deg_e7; int32_t hmsl_mm; uint32_t hacc_mm;
    uint32_t vacc_mm; int32_t vel_n_mm_s; int32_t vel_e_mm_s; int32_t vel_d_mm_s;
    int32_t heading_motion_deg_e5; uint16_t year; uint8_t month; uint8_t day;
    uint8_t hour; uint8_t minute; uint8_t second;
} APP_GPS_Status;
typedef struct {
    APP_Baro_Status status; int32_t raw_status; int32_t coef_status; uint8_t scaled_valid;
    int32_t pressure_raw; int32_t temperature_raw; int32_t pressure_pa;
    int32_t temperature_cdeg; uint8_t prs_cfg; uint8_t tmp_cfg; uint8_t meas_cfg;
    uint8_t cfg_reg; uint8_t int_sts; uint8_t fifo_sts; uint8_t raw_regs[14]; uint8_t id;
} APP_Baro_Snapshot;
typedef struct {
    uint8_t initialized; uint32_t predict_count; uint32_t flow_update_count;
    uint32_t flow_reject_count; uint32_t flow_skip_count; float last_nis;
    float last_gate_nis; float last_innovation_m_s[2]; float last_flow_noise_m_s;
    float vel_m_s[2]; float accel_bias_m_s2[2]; float covariance_diag[4];
} DRV_NAV_EKF_Diagnostics;
typedef struct { uint8_t stack_overflow_seen; char stack_overflow_task[32]; uint8_t malloc_failed_seen; uint32_t malloc_failed_count; } APP_DiagFaultInfo;

typedef enum {
    APP_AIWB2_STATE_START_DELAY = 0, APP_AIWB2_STATE_WAIT_BOOT_CONNECT,
    APP_AIWB2_STATE_ESCAPE_BEFORE, APP_AIWB2_STATE_WAIT_PROBE,
    APP_AIWB2_STATE_SEND_COMMAND, APP_AIWB2_STATE_WAIT_COMMAND,
    APP_AIWB2_STATE_ESCAPE_AFTER, APP_AIWB2_STATE_WAIT_TRANSPARENT_OK,
    APP_AIWB2_STATE_SOCKET_READY, APP_AIWB2_STATE_TRANSPARENT,
    APP_AIWB2_STATE_RETRY_DELAY
} APP_AiWB2_State;
typedef enum {
    BSP_ICM42688_INIT_STAGE_NONE = 0, BSP_ICM42688_INIT_STAGE_BANK_SELECT,
    BSP_ICM42688_INIT_STAGE_RESET, BSP_ICM42688_INIT_STAGE_WHO_AM_I,
    BSP_ICM42688_INIT_STAGE_GYRO_CONFIG, BSP_ICM42688_INIT_STAGE_ACCEL_CONFIG,
    BSP_ICM42688_INIT_STAGE_FILTER_CONFIG, BSP_ICM42688_INIT_STAGE_PWR_MGMT,
    BSP_ICM42688_INIT_STAGE_SIGNAL_RESET, BSP_ICM42688_INIT_STAGE_READY
} BSP_ICM42688_InitStage;
#define BSP_ICM42688_WHO_AM_I_VALUE 0x47U
#define BSP_SPL06_ID_VALUE 0x10U
#define BSP_SPL06_OK 0

#define APP_PROTO_MSG_CAPS_RECORD 1U
#define APP_PROTO_MSG_WIFI_RECORD 2U
#define APP_PROTO_MSG_RTOS_RECORD 3U
#define APP_PROTO_MSG_MODULES_SUMMARY 4U
#define APP_PROTO_MSG_HW_FLASH 5U
#define APP_PROTO_MSG_HW_BARO 6U
#define APP_PROTO_MSG_HW_IMU 7U
#define APP_PROTO_MSG_GPS_RECORD 8U
#define APP_PROTO_MSG_MAG_RECORD 9U
#define APP_PROTO_MSG_STATUS_FLASH 10U
#define APP_PROTO_MSG_STATUS_BARO 11U
#define APP_PROTO_MSG_STATUS_IMU 12U
#define APP_PROTO_MSG_UART_STATS 13U

void APP_Flash_GetStatus(APP_Flash_Status *);
void APP_Baro_GetStatus(APP_Baro_Status *);
void APP_Baro_ReadSnapshot(APP_Baro_Snapshot *);
void APP_IMU_GetStatus(APP_IMU_Status *);
void APP_OpticalFlow_GetStatus(APP_OPTICAL_FLOW_Status *);
const char *APP_OpticalFlow_VelSourceName(int);
void APP_MAG_GetStatus(APP_MAG_Status *);
const char *APP_MAG_GetTypeName(int);
void APP_GPS_GetStatus(APP_GPS_Status *);
void APP_NavEstimator_GetVelocityEKF(DRV_NAV_EKF_Diagnostics *);
void APP_Diag_GetFaultInfo(APP_DiagFaultInfo *);
void APP_UART_GetStats(uint32_t *, uint32_t *, uint32_t *, uint32_t *);
void APP_UART_GetRxEventStats(uint32_t *, uint32_t *, uint32_t *);
uint32_t APP_USB_CDC_GetTxSent(void);
uint32_t APP_USB_CDC_GetTxDropped(void);

APP_AiWB2_State APP_AiWB2_GetState(void);
uint8_t APP_AiWB2_IsTransparent(void);
uint32_t APP_AiWB2_GetRetryCount(void);
int32_t APP_AiWB2_GetLastSocketError(void);
uint8_t APP_AiWB2_IsPowerRecycleActive(void);
uint32_t APP_AiWB2_GetDeadlineRemainingMs(void);
uint8_t APP_AiWB2_IsProvisionActive(void);
uint32_t APP_AiWB2_GetCommandIndex(void);
uint32_t APP_AiWB2_GetCommandCount(void);
uint8_t BSP_AiWB2_IsEnabled(void);
uint8_t BSP_AiWB2_GetLastWrittenState(void);
uint32_t BSP_AiWB2_GetWriteCount(void);
uint32_t HAL_GetTick(void);

typedef void *osThreadId_t;
typedef void *osMessageQueueId_t;
typedef void *TaskHandle_t;
typedef uint32_t UBaseType_t;
extern osMessageQueueId_t uartTxQueueHandle;
extern osMessageQueueId_t backgroundReqQueueHandle;
extern osMessageQueueId_t backgroundRespQueueHandle;
extern osThreadId_t StabilizerHandle;
extern osThreadId_t VOFA_TaskHandle;
extern osThreadId_t SensorTaskHandle;
extern osThreadId_t messageTaskHandle;
extern osThreadId_t UARTTaskHandle;
extern osThreadId_t backgroundTaskHandle;
uint32_t osMessageQueueGetCount(osMessageQueueId_t);
uint32_t osMessageQueueGetCapacity(osMessageQueueId_t);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t);
/* 2026-09-11: RTOS? also reports the scheduler state per task. */
typedef enum { eRunning = 0, eReady, eBlocked, eSuspended, eDeleted, eInvalid } eTaskState;
eTaskState eTaskGetState(TaskHandle_t);
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);

#endif
"""

CORE_HARNESS = r"""
#include "app_control_internal.h"
#include "app_messages.h"
#include "app_tasks.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

osMessageQueueId_t uartTxQueueHandle = (osMessageQueueId_t)1;
static APP_UART_TxMessage captured;
static uint32_t put_count;
static uint32_t usb_timeout_ms;
static uint32_t notify_count;
static uint32_t maint_count;
static uint8_t maint_active;

osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *message,
                             uint8_t priority, uint32_t timeout)
{
    (void)queue; (void)priority; (void)timeout;
    captured = *(const APP_UART_TxMessage *)message;
    put_count++;
    return osOK;
}

osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *message,
                             uint8_t *priority, uint32_t timeout)
{
    (void)queue; (void)message; (void)priority; (void)timeout;
    return osOK;
}

uint8_t APP_IMU_Capture_IsExportActive(void) { return 0U; }
uint8_t APP_USB_CDC_Write(const uint8_t *data, uint16_t length,
                          uint32_t timeout_ms)
{
    (void)data; (void)length;
    usb_timeout_ms = timeout_ms;
    return 1U;
}
void APP_UART_NotifyTxPending(void) { notify_count++; }
/*
 * 2026-09-11: the Bluetooth link is only mirrored to while it is in use.
 * Returning 0 keeps this harness observing the pre-existing paths (USB mirror
 * plus the UART queue); the mirror itself is covered by its own contract test.
 */
uint8_t APP_MaintUART_IsLinkActive(void)
{
    return 0U;
}

void APP_MaintUART_Write(const char *text, uint16_t length)
{
    (void)text; (void)length;
    maint_count++;
}
uint8_t app_control_internal_maint_output_active(void) { return maint_active; }

int main(void)
{
    char line[] = "CMD value=42 signed=-17";
    char *tokens[4] = {0};
    uint32_t u32 = 0U;
    int32_t i32 = 0;
    const uint8_t crc_input[] = "123456789";

    CHECK(app_control_tokenize(line, tokens, 4U) == 3U, 1);
    CHECK(strcmp(tokens[0], "CMD") == 0, 2);
    CHECK(strcmp(app_control_token_value(tokens, 3U, "value"), "42") == 0, 3);
    CHECK(app_control_token_value(tokens, 3U, "missing") == NULL, 4);
    CHECK(app_control_parse_u32("429", &u32) == 1U && u32 == 429U, 5);
    CHECK(app_control_parse_u32("42x", &u32) == 0U, 6);
    CHECK(app_control_parse_i32("-17", &i32) == 1U && i32 == -17, 7);
    CHECK(app_control_parse_i32("", &i32) == 0U, 8);
    CHECK(app_control_crc32(crc_input, 9U) == 0xCBF43926UL, 9);

    app_control_queue_proto_text(0x2222U, "VALUE %lu\r\n", (unsigned long)u32);
    CHECK(captured.function == 0x2222U, 10);
    CHECK(strcmp(captured.text, "VALUE 429\r\n") == 0, 11);
    CHECK(usb_timeout_ms == 10U, 12);
    CHECK(put_count == 1U && notify_count == 1U && maint_count == 0U, 13);

    maint_active = 1U;
    app_control_queue_proto_text(0x2223U, "MAINT\r\n");
    CHECK(put_count == 1U && maint_count == 1U, 14);
    return 0;
}
"""


def _c_function_body(source: str, name: str) -> str:
    pattern = re.compile(
        rf"^(?:static\s+)?(?:void|uint8_t|uint32_t|int32_t|const char \*\s*)\s*"
        rf"{re.escape(name)}\s*\(",
        re.MULTILINE,
    )
    matches = list(pattern.finditer(source))
    assert matches, name
    match = matches[-1]
    brace = source.index("{", match.start())
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError(f"unterminated C function: {name}")


def _write_core_stubs(stub_dir: Path) -> None:
    stub_dir.mkdir()
    (stub_dir / "app_imu_capture.h").write_text(
        "#include <stdint.h>\nuint8_t APP_IMU_Capture_IsExportActive(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_maint_uart.h").write_text(
        "#include <stdint.h>\n"
        "void APP_MaintUART_Write(const char *, uint16_t);\n"
        "uint8_t APP_MaintUART_IsLinkActive(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_messages.h").write_text(
        "#include <stdint.h>\n"
        "typedef struct { uint16_t function; uint16_t length; char text[256]; } "
        "APP_UART_TxMessage;\n",
        encoding="ascii",
    )
    (stub_dir / "app_tasks.h").write_text(
        "#include <stdint.h>\n"
        "typedef void *osMessageQueueId_t; typedef int32_t osStatus_t;\n"
        "#define osOK 0\n"
        "extern osMessageQueueId_t uartTxQueueHandle;\n"
        "osStatus_t osMessageQueuePut(osMessageQueueId_t,const void*,uint8_t,uint32_t);\n"
        "osStatus_t osMessageQueueGet(osMessageQueueId_t,void*,uint8_t*,uint32_t);\n",
        encoding="ascii",
    )
    (stub_dir / "app_uart.h").write_text(
        "void APP_UART_NotifyTxPending(void);\n", encoding="ascii"
    )
    (stub_dir / "app_usb_cdc.h").write_text(
        "#include <stdint.h>\n"
        "uint8_t APP_USB_CDC_Write(const uint8_t*,uint16_t,uint32_t);\n",
        encoding="ascii",
    )


def _body_after_signature(source: str, signature: str) -> str:
    start = source.rindex(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError(f"unterminated C function after {signature}")


def _check_app_control_step_b() -> None:
    control = CONTROL.read_text(encoding="utf-8")
    imucal = IMUCAL_CMD.read_text(encoding="utf-8") if IMUCAL_CMD.is_file() else ""
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    for name, expected_hash in STEP_B_LEGACY_BODY_SHA256.items():
        body = _c_function_body(control, name)
        assert hashlib.sha256(body.encode()).hexdigest() == expected_hash
    for signature, expected_body in STEP_B_ACCESSOR_BODIES.items():
        owner = (
            imucal
            if any(name in signature for name in (
                "imucal_upload_state", "imucal_applied", "imucal_commit_pending"
            ))
            else control
        )
        assert _body_after_signature(owner, signature) == expected_body
        assert signature in internal
    assert not re.search(r"\bextern\b.*\bcontrol_(?:imucal|imuframe)", internal)


def _write_servocal_stubs(stub_dir: Path) -> None:
    (stub_dir / "app_control.h").write_text(
        "void APP_Control_QueueText(const char *, ...);\n", encoding="ascii"
    )
    (stub_dir / "app_acceptance.h").write_text(
        "#include <stdint.h>\nuint8_t APP_Acceptance_IsActive(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_proto.h").write_text(
        "#define APP_PROTO_MSG_SERVO_CAL 0x2225U\n", encoding="ascii"
    )
    (stub_dir / "app_sensor.h").write_text(
        "#include <stdint.h>\nuint8_t APP_Sensor_GetFluOrientation(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_stabilizer.h").write_text(
        "#include <stdint.h>\n"
        "typedef struct { uint32_t value[16]; } StabilizerValidationImuSnapshot;\n"
        "uint8_t APP_Stabilizer_IsServoCalibrationCandidateArmLocked(void);\n"
        "void APP_Stabilizer_SetServoCalibrationCandidateArmLock(uint8_t);\n",
        encoding="ascii",
    )
    (stub_dir / "drv_coax_ctrl.h").write_text(
        "#include <stdint.h>\n"
        "#define DRV_COAX_CTRL_SERVO_COUNT 2U\n"
        "#define DRV_COAX_CTRL_SERVO_ALPHA_INDEX 0U\n"
        "#define DRV_COAX_CTRL_SERVO_BETA_INDEX 1U\n"
        "typedef struct { uint16_t center_us[2]; uint16_t min_us[2]; "
        "uint16_t max_us[2]; int8_t pulse_sign[2]; } DRV_COAX_CTRL_ServoCalibration;\n"
        "void DRV_COAX_CTRL_GetDefaultServoCalibration(DRV_COAX_CTRL_ServoCalibration*);\n"
        "uint8_t DRV_COAX_CTRL_ValidateServoCalibration(const DRV_COAX_CTRL_ServoCalibration*);\n",
        encoding="ascii",
    )
    (stub_dir / "svc_param.h").write_text(
        "#include <stdint.h>\n"
        "typedef enum { SVC_PARAM_STATUS_OK = 0 } SVC_ParamStatus;\n"
        "uint8_t SVC_Param_IsDirty(void);\n"
        "SVC_ParamStatus SVC_Param_SetBlob(const uint8_t*,uint32_t);\n"
        "uint32_t SVC_Param_RequestSaveBlob(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_flight_calibration.h").write_text(
        "#include <stdint.h>\n"
        "typedef struct { uint32_t calibration_generation; uint8_t bytes[156]; } APP_FlightCalibration;\n"
        "typedef struct { APP_FlightCalibration calibration; uint32_t generation; } APP_FlightCalibrationSnapshot;\n"
        "typedef enum { APP_FLIGHT_CAL_UPLOAD_EMPTY = 0, APP_FLIGHT_CAL_UPLOAD_READY = 3 } APP_FlightCalibrationUploadState;\n"
        "typedef struct { APP_FlightCalibrationUploadState state; } APP_FlightCalibrationUpload;\n"
        "uint8_t APP_FlightCalibration_ReadActive(APP_FlightCalibrationSnapshot*);\n"
        "uint8_t APP_FlightCalibration_BuildServoMechanical(const APP_FlightCalibration*,void*);\n"
        "uint8_t APP_FlightCalibration_UpdateServoMechanical(APP_FlightCalibration*,const void*);\n"
        "uint8_t APP_FlightCalibration_PublishPreview(const APP_FlightCalibration*);\n"
        "uint32_t APP_FlightCalibration_Encode(const APP_FlightCalibration*,uint8_t*,uint32_t);\n",
        encoding="ascii",
    )


def _check_app_control_step_c(tmp_path: Path) -> None:
    assert SERVOCAL_CMD.is_file()
    legacy = CONTROL.read_text(encoding="utf-8")
    servocal = SERVOCAL_CMD.read_text(encoding="utf-8")
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    for name, expected_hash in STEP_C_BODY_SHA256.items():
        assert hashlib.sha256(_c_function_body(servocal, name).encode()).hexdigest() == expected_hash
        assert not re.search(
            rf"^(?:static\s+)?(?:void|uint8_t|uint32_t|const char \*\s*)\s*"
            rf"{re.escape(name)}\s*\(",
            legacy,
            re.MULTILINE,
        ), name
    for name in STEP_C_STATE_NAMES:
        assert re.search(rf"^static .*\b{re.escape(name)}(?:\b|;)", servocal, re.MULTILINE)
        assert name not in legacy
    assert len(servocal.splitlines()) <= 800
    assert "extern" not in servocal
    assert STEP_C_PERSISTED_FRAGMENT in servocal
    assert STEP_C_INIT_FRAGMENT in servocal
    sync = _c_function_body(legacy, "app_control_imuframe_sync_param")
    assert sync.index("app_control_imucal_clear_candidate();") < sync.index(
        "app_cmd_servocal_on_persisted(&calibration);"
    ) < sync.index("control_imuframe_confirmed_code = orientation_code;")
    report = _c_function_body(servocal, "app_control_report_servocal")
    handler = _c_function_body(servocal, "app_control_handle_servocal")
    assert report.index("app_control_imuframe_sync_param();") < report.index("memset(&active")
    assert handler.index("app_control_imuframe_sync_param();") < handler.index('strcmp(tokens[0], "SERVOCAL?")')
    for call in (
        "app_cmd_servocal_init();",
        "app_cmd_servocal_on_persisted(&calibration);",
        "app_control_service_servocal();",
        "app_control_handle_servocal(tokens, count);",
    ):
        assert call in legacy
    assert "app_cmd_servocal_is_busy()" in IMUCAL_CMD.read_text(encoding="utf-8")
    for declaration in (
        "void app_control_handle_servocal(char **tokens, uint32_t count);",
        "void app_control_service_servocal(void);",
        "void app_cmd_servocal_init(void);",
        "void app_cmd_servocal_on_persisted(const void *record);",
        "uint8_t app_cmd_servocal_is_busy(void);",
    ):
        assert declaration in internal

    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub_dir = tmp_path / "servocal_stubs"
    stub_dir.mkdir()
    _write_servocal_stubs(stub_dir)
    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub_dir}",
            f"-I{ROOT / 'App' / 'Inc'}",
            "-c",
            str(SERVOCAL_CMD),
            "-o",
            str(tmp_path / "app_cmd_servocal.o"),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def _write_imucal_stubs(stub_dir: Path) -> None:
    stub_dir.mkdir()
    (stub_dir / "app_control.h").write_text(
        "void APP_Control_QueueText(const char *, ...);\n", encoding="ascii"
    )
    (stub_dir / "app_firmware_identity.h").write_text(
        "#include <stdint.h>\n"
        "typedef struct { uint32_t image_crc32; uint32_t image_size; } APP_FirmwareIdentity;\n"
        "uint8_t APP_FirmwareIdentity_Get(APP_FirmwareIdentity*);\n",
        encoding="ascii",
    )
    (stub_dir / "app_proto.h").write_text(
        "#define APP_PROTO_MSG_IMU_CAL 0x2221U\n", encoding="ascii"
    )
    (stub_dir / "app_sensor.h").write_text(
        "#include <stdint.h>\n"
        "#define APP_SENSOR_FLU_ORIENTATION_LEGACY 255U\n"
        "typedef struct { uint32_t value; } DRV_IMU_Calibration;\n"
        "uint8_t APP_Sensor_GetFluOrientation(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_stabilizer.h").write_text(
        "#include <stdint.h>\n"
        "typedef struct { uint32_t sequence; uint32_t calibration_generation; } StabilizerValidationImuSnapshot;\n"
        "uint8_t APP_Stabilizer_IsImuCalibrationCandidateArmLocked(void);\n"
        "void APP_Stabilizer_SetImuCalibrationCandidateArmLock(uint8_t);\n",
        encoding="ascii",
    )
    (stub_dir / "main.h").write_text(
        "#include <stdint.h>\nuint32_t HAL_GetTick(void);\n", encoding="ascii"
    )
    (stub_dir / "svc_param.h").write_text(
        "#include <stdint.h>\n"
        "typedef enum { SVC_PARAM_STATUS_OK = 0 } SVC_ParamStatus;\n"
        "uint8_t SVC_Param_IsDirty(void);\n"
        "SVC_ParamStatus SVC_Param_GetBlob(uint8_t*,uint32_t,uint32_t*);\n"
        "SVC_ParamStatus SVC_Param_SetBlob(const uint8_t*,uint32_t);\n"
        "uint32_t SVC_Param_RequestSaveBlob(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_flight_calibration.h").write_text(
        "#include <stdint.h>\n"
        "#define APP_FLIGHT_CAL_VALID_ORIENTATION 0x01U\n"
        "typedef struct { uint32_t base_generation; uint8_t orientation_code; "
        "uint8_t valid_mask; } APP_FlightCalibrationV1Candidate;\n"
        "typedef struct { uint8_t schema; uint8_t frame_contract; "
        "uint32_t calibration_generation; uint8_t valid_mask; uint8_t orientation_code; "
        "float accel_bias[3]; float accel_correction[3][3]; float gyro_bias_ref[3]; "
        "float gyro_temp_slope[3]; float reference_temp_c; uint8_t pad[64]; } APP_FlightCalibration;\n"
        "typedef struct { APP_FlightCalibration calibration; uint32_t generation; } APP_FlightCalibrationSnapshot;\n"
        "typedef enum { APP_FLIGHT_CAL_UPLOAD_EMPTY = 0, APP_FLIGHT_CAL_UPLOAD_READY = 3 } APP_FlightCalibrationUploadState;\n"
        "typedef struct { APP_FlightCalibrationV1Candidate candidate; "
        "uint32_t received_size; uint32_t expected_size; APP_FlightCalibrationUploadState state; } APP_FlightCalibrationUpload;\n"
        "typedef enum { APP_FLIGHT_CAL_TRANSFER_OK = 0 } APP_FlightCalibrationTransferStatus;\n"
        "typedef enum { APP_FLIGHT_CAL_DECODE_INVALID = 0 } APP_FlightCalibrationDecodeStatus;\n"
        "struct DRV_IMU_Calibration;\n"
        "uint8_t APP_FlightCalibration_ReadActive(APP_FlightCalibrationSnapshot*);\n"
        "uint8_t APP_FlightCalibration_BuildImuCalibration(const APP_FlightCalibration*,uint8_t,void*);\n"
        "const char *APP_FlightCalibration_UploadStateText(APP_FlightCalibrationUploadState);\n"
        "void APP_FlightCalibration_UploadReset(APP_FlightCalibrationUpload*);\n"
        "APP_FlightCalibrationTransferStatus APP_FlightCalibration_UploadBegin(APP_FlightCalibrationUpload*,uint32_t,uint32_t,uint32_t);\n"
        "APP_FlightCalibrationTransferStatus APP_FlightCalibration_UploadDataHex(APP_FlightCalibrationUpload*,uint32_t,const char*,uint32_t);\n"
        "APP_FlightCalibrationTransferStatus APP_FlightCalibration_UploadEnd(APP_FlightCalibrationUpload*,uint32_t);\n"
        "uint8_t APP_FlightCalibration_UploadExpire(APP_FlightCalibrationUpload*,uint32_t);\n"
        "const char *APP_FlightCalibration_TransferStatusText(APP_FlightCalibrationTransferStatus);\n"
        "uint8_t APP_FlightCalibration_MergeV1Candidate(const APP_FlightCalibration*,const APP_FlightCalibrationV1Candidate*,APP_FlightCalibration*);\n"
        "uint8_t APP_FlightCalibration_PublishPreview(const APP_FlightCalibration*);\n"
        "APP_FlightCalibrationDecodeStatus APP_FlightCalibration_Decode(const uint8_t*,uint32_t,APP_FlightCalibration*);\n"
        "uint32_t APP_FlightCalibration_Encode(const APP_FlightCalibration*,uint8_t*,uint32_t);\n",
        encoding="ascii",
    )


def _check_app_control_step_d1(tmp_path: Path) -> None:
    assert IMUCAL_CMD.is_file()
    legacy = CONTROL.read_text(encoding="utf-8")
    imucal = IMUCAL_CMD.read_text(encoding="utf-8")
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    for name, expected_hash in STEP_D1_BODY_SHA256.items():
        assert hashlib.sha256(_c_function_body(imucal, name).encode()).hexdigest() == expected_hash
        assert not re.search(
            rf"^(?:static\s+)?(?:void|uint8_t|uint32_t|const char \*\s*)\s*"
            rf"{re.escape(name)}\s*\(",
            legacy,
            re.MULTILINE,
        ), name
    for name in STEP_D1_STATE_NAMES:
        assert re.search(rf"^static .*\b{re.escape(name)}(?:\b|;)", imucal, re.MULTILINE)
        assert not re.search(rf"^static .*\b{re.escape(name)}(?:\b|;)", legacy, re.MULTILINE)
        assert re.search(rf"^#define\s+{re.escape(name)}\b", legacy, re.MULTILINE)
    assert len(imucal.splitlines()) <= 800
    assert "extern" not in imucal
    for name, expected_hash in STEP_D1_LEGACY_BODY_SHA256.items():
        body = _c_function_body(legacy, name)
        if name == "APP_Control_Init":
            body = body.replace(
                "app_cmd_rcmap_apply_config(NULL);",
                "app_control_apply_rc_config(NULL);",
            )
        assert hashlib.sha256(body.encode()).hexdigest() == expected_hash
    handler = _c_function_body(imucal, "app_control_handle_imucal")
    assert handler.index("app_control_imuframe_sync_param();") < handler.index('strcmp(tokens[0], "IMUCAL?")')
    assert "app_cmd_servocal_is_busy()" in handler
    assert "APP_CONTROL_IMUCAL_SNAPSHOT_MAX_AGE_US 100000ULL" in legacy
    assert "APP_CONTROL_IMUCAL_ESC_SAFE_MAX_US 1100U" in legacy
    for declaration in (
        "void app_control_handle_imucal(char **tokens, uint32_t count);",
        "void app_control_service_imucal(void);",
        "void *app_cmd_imucal_upload_slot(void);",
        "uint32_t *app_control_internal_imucal_apply_sequence_slot(void);",
    ):
        assert declaration in internal

    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub_dir = tmp_path / "imucal_stubs"
    _write_imucal_stubs(stub_dir)
    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub_dir}",
            f"-I{ROOT / 'App' / 'Inc'}",
            "-c",
            str(IMUCAL_CMD),
            "-o",
            str(tmp_path / "app_cmd_imucal.o"),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def _write_rcmap_stubs(stub_dir: Path) -> None:
    stub_dir.mkdir()
    (stub_dir / "app_flash_service.h").write_text(
        "typedef enum { APP_FLASH_SERVICE_OK = 0, APP_FLASH_SERVICE_ERROR = 1 } "
        "APP_FlashService_Status;\n",
        encoding="ascii",
    )
    (stub_dir / "app_stabilizer.h").write_text(
        "#include <stdint.h>\nuint8_t APP_Stabilizer_IsArmed(void);\n",
        encoding="ascii",
    )
    (stub_dir / "main.h").write_text(
        "#include <stdint.h>\nuint32_t HAL_GetTick(void);\n",
        encoding="ascii",
    )


def _check_app_control_step_d2(tmp_path: Path) -> None:
    assert RCMAP_CMD.is_file()
    legacy = CONTROL.read_text(encoding="utf-8")
    rcmap = RCMAP_CMD.read_text(encoding="utf-8")
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")

    for name, expected_hash in STEP_D2_BODY_SHA256.items():
        assert hashlib.sha256(_c_function_body(rcmap, name).encode()).hexdigest() == expected_hash, (
            f"{name} diverged from D2 parent {STEP_D2_PARENT_COMMIT}"
        )
    handler = _c_function_body(rcmap, "app_control_handle_rc_map")
    assert hashlib.sha256(handler.encode()).hexdigest() == STEP_D2_CHILD_HANDLER_SHA256
    assert handler.count(STEP_D2_DELEGATED_PERSIST_FRAGMENT) == 1
    parent_equivalent = handler.replace(
        STEP_D2_DELEGATED_PERSIST_FRAGMENT,
        STEP_D2_PARENT_PERSIST_FRAGMENT,
    )
    assert hashlib.sha256(parent_equivalent.encode()).hexdigest() == STEP_D2_PARENT_HANDLER_SHA256

    for name in (*STEP_D2_BODY_SHA256, "app_control_handle_rc_map"):
        assert not re.search(
            rf"^(?:static\s+)?(?:void|uint8_t)\s+{re.escape(name)}\s*\(",
            legacy,
            re.MULTILINE,
        ), name
    for name in ("control_rc_config", "control_rc_config_dirty"):
        assert re.search(rf"^static .*\b{re.escape(name)}(?:\b|;)", rcmap, re.MULTILINE)
        assert name not in legacy
    assert "#define APP_CONTROL_RC_FRESH_TIMEOUT_MS 500U" in rcmap
    assert "APP_CONTROL_RC_FRESH_TIMEOUT_MS" not in legacy
    assert len(rcmap.splitlines()) <= 800
    assert "extern" not in rcmap
    assert "control_config" not in rcmap
    assert "app_control_save_config" not in rcmap

    assert _body_after_signature(
        legacy,
        "uint8_t app_control_internal_commit_config_persist(void)",
    ) == STEP_D2_PERSIST_HELPER_BODY
    config_store = (ROOT / "App/Src/app_control_config_store.c").read_text(encoding="utf-8")
    # 6 → 7：v20 在记录尾部追加了机体模型块，于是多了一个 v19 迁移读取器，
    # 它同样要把旧记录里的 RC 配置应用过来。每加一条迁移链就多一次调用，
    # 这个数会随版本增长——它守的是"应用点数量可数、不会散落"，不是某个定值。
    assert (legacy + config_store).count("app_cmd_rcmap_apply_config(") == 7
    assert (legacy + config_store).count("app_cmd_rcmap_config()") == 1
    assert "app_control_report_rc_live();" in legacy
    assert "app_control_handle_rc_map(tokens, count);" in legacy
    assert "App/Src/app_cmd_rcmap.c" in cmake
    for declaration in (
        "void app_control_report_rc_live(void);",
        "void app_control_handle_rc_map(char *tokens[], uint32_t count);",
        "void app_cmd_rcmap_apply_config(const void *config);",
        "const void *app_cmd_rcmap_config(void);",
        "uint8_t app_control_internal_commit_config_persist(void);",
    ):
        assert declaration in internal
    flash_driver = (ROOT / "Driver" / "Inc" / "drv_gd25q32.h").read_text(
        encoding="utf-8"
    )
    assert "DRV_GD25Q32_OK = 0" in flash_driver
    assert "DRV_GD25Q32_DMA_ERROR\n} DRV_GD25Q32_Status;" in flash_driver

    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub_dir = tmp_path / "rcmap_stubs"
    _write_rcmap_stubs(stub_dir)
    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub_dir}",
            f"-I{ROOT / 'App' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            "-c",
            str(RCMAP_CMD),
            "-o",
            str(tmp_path / "app_cmd_rcmap.o"),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def _write_flow_stubs(stub_dir: Path) -> None:
    stub_dir.mkdir()
    (stub_dir / "app_control.h").write_text(
        "void APP_Control_QueueText(const char *, ...);\n", encoding="ascii"
    )
    (stub_dir / "app_optical_flow.h").write_text(
        "void APP_OpticalFlow_Report(void);\n", encoding="ascii"
    )
    (stub_dir / "app_stabilizer.h").write_text(
        "#include <stdint.h>\n"
        "typedef struct { uint32_t sample_ms; uint16_t frame_contract; "
        "uint8_t orientation_code; uint8_t valid; "
        "float sensor_velocity_flu_m_s[2]; float optical_rot_comp_flu_m_s[2]; "
        "float offset_rot_comp_flu_m_s[2]; float corrected_velocity_flu_m_s[2]; "
        "} StabilizerFlowCompensationSnapshot;\n"
        "uint8_t APP_Stabilizer_ReadFlowCompensationSnapshot("
        "StabilizerFlowCompensationSnapshot *);\n",
        encoding="ascii",
    )
    # R-M5-5：FLOW ZERO 走 Service 的里程计归零接口，这里只需要它的三个符号。
    (stub_dir / "svc_flow_nav.h").write_text(
        "#include <stdint.h>\n"
        "void SVC_FlowNav_ResetDisplacement(void);\n"
        "void SVC_FlowNav_GetDisplacement(float *, float *);\n"
        "uint32_t SVC_FlowNav_GetIntegratedStepCount(void);\n",
        encoding="ascii",
    )
    (stub_dir / "bsp_optical_flow.h").write_text(
        "#include <stdint.h>\n"
        "typedef int BSP_OPTICAL_FLOW_StatusCode;\n"
        "BSP_OPTICAL_FLOW_StatusCode BSP_OPTICAL_FLOW_TransmitRaw("
        "const uint8_t *,uint16_t,uint32_t);\n"
        "uint16_t BSP_OPTICAL_FLOW_ReceiveRaw(uint8_t *,uint16_t,uint32_t);\n"
        "BSP_OPTICAL_FLOW_StatusCode BSP_OPTICAL_FLOW_TransceiveRaw("
        "const uint8_t *,uint16_t,uint8_t *,uint16_t,uint16_t *,uint32_t);\n",
        encoding="ascii",
    )


def _check_app_control_step_d3(tmp_path: Path) -> None:
    assert FLOW_CMD.is_file()
    legacy = CONTROL.read_text(encoding="utf-8")
    flow = FLOW_CMD.read_text(encoding="utf-8")
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")

    for name, expected_hash in STEP_D3_BODY_SHA256.items():
        assert hashlib.sha256(_c_function_body(flow, name).encode()).hexdigest() == expected_hash, (
            f"{name} diverged from D3 parent {STEP_D3_PARENT_COMMIT}"
        )
        assert not re.search(
            rf"^(?:static\s+)?(?:void|uint8_t)\s+{re.escape(name)}\s*\(",
            legacy,
            re.MULTILINE,
        ), name
    assert hashlib.sha256(
        _c_function_body(legacy, "app_control_acceptance_milli").encode()
    ).hexdigest() == STEP_D3_ACCEPTANCE_BODY_SHA256
    assert _body_after_signature(
        legacy,
        "int32_t app_control_internal_acceptance_milli(float value)",
    ) == STEP_D3_ACCESSOR_BODY
    assert (
        "#define app_control_acceptance_milli \\\n"
        "    app_control_internal_acceptance_milli"
    ) in flow
    assert "#define APP_CONTROL_FLOW_RAW_MAX_BYTES 32U" in flow
    assert "APP_CONTROL_FLOW_RAW_MAX_BYTES" not in legacy
    assert len(flow.splitlines()) <= 800
    assert "extern" not in flow
    assert "App/Src/app_cmd_flow.c" in cmake
    assert "app_control_report_flow();" in legacy
    assert "app_control_handle_flow(tokens, count);" in legacy
    for declaration in (
        "int32_t app_control_internal_acceptance_milli(float value);",
        "void app_control_handle_flow(char **tokens, uint32_t count);",
        "void app_control_report_flow(void);",
    ):
        assert declaration in internal

    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub_dir = tmp_path / "flow_stubs"
    _write_flow_stubs(stub_dir)
    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub_dir}",
            f"-I{ROOT / 'App' / 'Inc'}",
            "-c",
            str(FLOW_CMD),
            "-o",
            str(tmp_path / "app_cmd_flow.o"),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def _write_d4_stubs(stub_dir: Path) -> None:
    stub_dir.mkdir()
    (stub_dir / "d4_stubs.h").write_text(D4_STUB_HEADER, encoding="ascii")
    headers = {
        "app_control.h", "app_aiwb2.h", "app_baro.h", "app_diag.h",
        "app_control_config_store.h", "app_flash_service.h",
        "app_flash.h", "app_gps.h", "app_mag.h", "app_nav_estimator.h",
        "app_optical_flow.h", "app_proto.h", "app_sensor.h", "app_tasks.h",
        "app_uart.h", "app_usb_cdc.h", "bsp_aiwb2_power.h", "bsp_baro.h",
        "bsp_imu.h", "main.h", "FreeRTOS.h", "task.h",
    }
    for header in headers:
        (stub_dir / header).write_text(
            '#include "d4_stubs.h"\n', encoding="ascii"
        )


def _check_app_control_step_d4(tmp_path: Path) -> None:
    assert SYSTEM_CMD.is_file()
    assert DIAG_CMD.is_file()
    legacy = CONTROL.read_text(encoding="utf-8")
    system = SYSTEM_CMD.read_text(encoding="utf-8")
    diag = DIAG_CMD.read_text(encoding="utf-8")
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")

    for owner, expected in (
        (system, STEP_D4_SYSTEM_BODY_SHA256),
        (diag, STEP_D4_DIAG_BODY_SHA256),
    ):
        for name, expected_hash in expected.items():
            assert hashlib.sha256(_c_function_body(owner, name).encode()).hexdigest() == expected_hash, (
                f"{name} diverged from D4 parent {STEP_D4_PARENT_COMMIT}"
            )
            assert not re.search(
                rf"^(?:static\s+)?(?:void|uint8_t|uint32_t|int32_t|const char \*\s*)\s*"
                rf"{re.escape(name)}\s*\(",
                legacy,
                re.MULTILINE,
            ), name
    for name, expected_hash in STEP_D4_PROTECTED_LEGACY_BODY_SHA256.items():
        assert hashlib.sha256(_c_function_body(legacy, name).encode()).hexdigest() == expected_hash
        assert not re.search(
            rf"^(?:static\s+)?(?:void|uint8_t|uint32_t|const char \*\s*)\s*"
            rf"{re.escape(name)}\s*\(",
            system + diag,
            re.MULTILINE,
        ), name

    assert len(system.splitlines()) <= 800
    assert len(diag.splitlines()) <= 800
    assert "extern control_" not in system + diag + internal
    assert (
        "#define control_config \\\n"
        "    (*(const APP_ControlConfig *)app_control_internal_config_view())"
    ) in system
    assert _body_after_signature(
        legacy,
        "const void *app_control_internal_config_view(void)",
    ) == "{\n    return &control_config;\n}"
    assert not re.search(
        r"^APP_ControlConfig \*app_control_internal_config_view",
        legacy + internal,
        re.MULTILINE,
    )

    delegate_bodies = {
        "const char *app_control_internal_imu_stage_name(uint8_t stage)": (
            "{\n    return app_control_imu_stage_name(stage);\n}"
        ),
        "uint8_t app_control_internal_flash_ok(const void *status)": (
            "{\n    return app_control_flash_ok((const APP_Flash_Status *)status);\n}"
        ),
        "const char *app_control_internal_flash_stage(const void *status)": (
            "{\n    return app_control_flash_stage((const APP_Flash_Status *)status);\n}"
        ),
        "uint8_t app_control_internal_baro_ok(const void *status)": (
            "{\n    return app_control_baro_ok((const APP_Baro_Status *)status);\n}"
        ),
        "const char *app_control_internal_baro_stage(const void *status)": (
            "{\n    return app_control_baro_stage((const APP_Baro_Status *)status);\n}"
        ),
        "const char *app_control_internal_aiwb2_state_name(uint32_t state)": (
            "{\n    return app_control_aiwb2_state_name((APP_AiWB2_State)state);\n}"
        ),
        "uint8_t app_control_internal_parse_u32_auto(const char *text, uint32_t *value)": (
            "{\n    return app_control_parse_u32_auto(text, value);\n}"
        ),
    }
    for signature, body in delegate_bodies.items():
        assert _body_after_signature(diag, signature) == body
        assert signature + ";" in internal

    for declaration in (
        "void app_control_report_caps(void);",
        "void app_control_report_wifi(void);",
        "void app_control_report_rtos(void);",
        "void app_control_report_modules(void);",
        "void app_control_report_status(void);",
        "void app_control_handle_req(char **tokens, uint32_t count);",
        "const void *app_control_internal_config_view(void);",
    ):
        assert declaration in internal
    for call in (
        "app_control_report_modules();",
        "app_control_report_caps();",
        "app_control_handle_req(tokens, count);",
        "app_control_report_status();",
        "app_control_report_rtos();",
        "app_control_report_wifi();",
    ):
        assert call in legacy
    assert "APP_Control_ReportUartStats(" not in legacy
    assert "APP_Control_ReportUartStats(" in (ROOT / "App/Src/app_uart.c").read_text(
        encoding="utf-8"
    )
    assert "App/Src/app_cmd_system.c" in cmake
    assert "App/Src/app_cmd_diag.c" in cmake

    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub_dir = tmp_path / "d4_stubs"
    _write_d4_stubs(stub_dir)
    for source, output in (
        (SYSTEM_CMD, "app_cmd_system.o"),
        (DIAG_CMD, "app_cmd_diag.o"),
    ):
        subprocess.run(
            [
                gcc,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-I{stub_dir}",
                f"-I{ROOT / 'App' / 'Inc'}",
                "-c",
                str(source),
                "-o",
                str(tmp_path / output),
            ],
            check=True,
            capture_output=True,
            text=True,
        )


def _check_app_control_step_a(tmp_path: Path) -> None:
    assert CONTROL_CORE.is_file()
    assert CONTROL_INTERNAL.is_file()
    control = CONTROL.read_text(encoding="utf-8")
    core = CONTROL_CORE.read_text(encoding="utf-8")
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")

    for name, expected_hash in STEP_A_BODY_SHA256.items():
        assert hashlib.sha256(_c_function_body(core, name).encode()).hexdigest() == expected_hash
        assert not re.search(
            rf"^(?:static\s+)?(?:void|uint8_t|uint32_t|const char \*\s*)\s*"
            rf"{re.escape(name)}\s*\(",
            control,
            re.MULTILINE,
        ), name
    assert '#include "app_control_internal.h"' in control
    assert "App/Src/app_control_core.c" in cmake
    assert "extern" not in core
    assert "#define APP_CONTROL_IMUCAL_SNAPSHOT_MAX_AGE_US 100000ULL" in control
    assert "#define APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS 10U" in internal
    assert re.findall(
        r"#define\s+APP_CONTROL_IMUCAL_SNAPSHOT_MAX_AGE_US\s+(\S+)",
        control + core + internal,
    ) == ["100000ULL"]
    assert re.findall(
        r"#define\s+APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS\s+(\S+)",
        control + core + internal,
    ) == ["10U"]
    assert "return control_maint_output_active;" in control

    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub_dir = tmp_path / "control_core_stubs"
    _write_core_stubs(stub_dir)
    harness = tmp_path / "control_core_harness.c"
    harness.write_text(CORE_HARNESS, encoding="ascii")
    executable = tmp_path / "control_core_harness.exe"
    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub_dir}",
            f"-I{ROOT / 'App' / 'Inc'}",
            str(CONTROL_CORE),
            str(harness),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def test_no_blocking() -> None:
    source = SERVO_CAL.read_text(encoding="utf-8")
    assert "APP_Control_QueueText(" not in source, (
        "app_servo_cal.c runs inside the 500Hz control loop; it must post "
        "notices instead of calling the blocking USB/UART text path"
    )
    assert '#include "app_control.h"' not in source, (
        "the control-loop module must not depend on app_control.h at all"
    )


def test_notice_buffer() -> None:
    source = SERVO_CAL.read_text(encoding="utf-8")
    assert "servo_cal_post_notice(" in source
    assert "servo_cal_notice_pending" in source
    assert "uint16_t APP_ServoCal_TakeNotice(" in source
    header = SERVO_CAL_HEADER.read_text(encoding="utf-8")
    assert "APP_ServoCal_TakeNotice" in header


def test_control_split(tmp_path: Path) -> None:
    source = CONTROL.read_text(encoding="utf-8")
    flush = source.find("APP_ServoCal_TakeNotice(")
    tick = source.find("static void app_control_tick_common(uint8_t emit_heartbeat)\n{")
    assert flush != -1, "app_control.c must flush the servo-cal notice"
    assert tick != -1, "app_control_tick_common definition not found"
    body = source[tick : source.find("\n}", tick)]
    assert "app_control_service_servo_cal_notice()" in body, (
        "app_control_tick_common must call the notice flush service"
    )
    _check_app_control_step_a(tmp_path)
    _check_app_control_step_b()
    _check_app_control_step_c(tmp_path)
    _check_app_control_step_d1(tmp_path)


def test_control_split_d2_rcmap(tmp_path: Path) -> None:
    _check_app_control_step_d2(tmp_path)


def test_control_split_d3_flow(tmp_path: Path) -> None:
    _check_app_control_step_d3(tmp_path)


def test_control_split_d4_system_and_diag(tmp_path: Path) -> None:
    _check_app_control_step_d4(tmp_path)

/*
 * Serial Studio - https://serial-studio.com/
 *
 * Copyright (C) 2020-2025 Alex Spataru <https://aspatru.com>
 *
 * This file is part of the proprietary feature set of Serial Studio
 * and is licensed under the Serial Studio Commercial License.
 *
 * Redistribution, modification, or use of this file in any form
 * is permitted only under the terms of a valid commercial license
 * obtained from the author.
 *
 * This file may NOT be used in any build distributed under the
 * GNU General Public License (GPL) unless explicitly authorized
 * by a separate commercial agreement.
 *
 * For license terms, see:
 * https://github.com/Serial-Studio/Serial-Studio/blob/master/LICENSE.md
 *
 * SPDX-License-Identifier: LicenseRef-SerialStudio-Commercial
 */

#pragma once

namespace Licensing {

/**
 * @brief Verifies every embedded license guard inside this binary; returns 0 on success.
 *
 * Release-gate self-test: runs all generated guard functions, the call-site dispatch,
 * and the feature-gate chain with a synthetic in-memory token, so a build with stale,
 * mangled, or tampered guard artifacts fails loudly instead of silently degrading
 * licensed features. Requires no activation state, UI, network, or disk access.
 */
[[nodiscard]] int runGuardSelfTest();

}  // namespace Licensing

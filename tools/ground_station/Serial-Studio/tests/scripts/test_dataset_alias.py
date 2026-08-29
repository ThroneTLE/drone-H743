"""
Contract tests for alias-addressed dataset reads (datasetGetRaw / datasetGetFinal).

The host bridge is not available in Node.js, so these exercise the API contract
against the TABLE_API_SHIM model of the C++ DataTableStore (see conftest.py): a
number selector is a uniqueId, a string selector is an alias, the two are never
coerced, and an unknown selector yields undefined. End-to-end correctness against
the real store (JS + Lua parity, the one-time warning) is covered by
tests/integration/test_dataset_alias.py.
"""

from .conftest import TABLE_API_SHIM, run_js


def _run(setup_js: str):
    """Run the shim + a setup snippet that console.logs a JSON result object."""
    return run_js(TABLE_API_SHIM + "\n" + setup_js)


# ---------------------------------------------------------------------------
# Alias and uniqueId resolve to the same value (AC1, JS half)
# ---------------------------------------------------------------------------


class TestAliasUniqueIdEquivalence:
    def test_raw_alias_equals_uniqueid(self):
        result = _run("""
            __defineDataset(128, 'ATAM1-CH1');
            __setDatasetRaw(128, 3.5);
            console.log(JSON.stringify({ byAlias: datasetGetRaw('ATAM1-CH1'),
                                         byUid:   datasetGetRaw(128) }));
            """)
        assert result["byAlias"] == 3.5
        assert result["byUid"] == 3.5

    def test_final_alias_equals_uniqueid(self):
        result = _run("""
            __defineDataset(7, 'speed');
            __setDatasetFinal(7, 42);
            console.log(JSON.stringify({ byAlias: datasetGetFinal('speed'),
                                         byUid:   datasetGetFinal(7) }));
            """)
        assert result["byAlias"] == 42
        assert result["byUid"] == 42

    def test_string_value_round_trips_via_alias(self):
        result = _run("""
            __defineDataset(3, 'status');
            __setDatasetRaw(3, 'OK');
            console.log(JSON.stringify({ v: datasetGetRaw('status') }));
            """)
        assert result["v"] == "OK"


# ---------------------------------------------------------------------------
# A string is always an alias, a number always a uniqueId (R3): "128" != 128
# ---------------------------------------------------------------------------


class TestStringNumberDiscrimination:
    def test_numeric_string_is_not_a_uniqueid(self):
        # Dataset uniqueId 128 exists but has NO alias "128": the string form must
        # not resolve to it.
        result = _run("""
            __defineDataset(128, null);
            __setDatasetRaw(128, 9);
            var s = datasetGetRaw('128');
            console.log(JSON.stringify({ byString: (s === undefined ? null : s),
                                         byNumber: datasetGetRaw(128) }));
            """)
        assert result["byNumber"] == 9
        assert result["byString"] is None

    def test_alias_and_uniqueid_can_differ(self):
        # An alias whose text equals another dataset's uniqueId resolves to its own
        # dataset, never the numeric one.
        result = _run("""
            __defineDataset(500, '128');
            __setDatasetRaw(500, 'alias-owner');
            __defineDataset(128, null);
            __setDatasetRaw(128, 'uid-owner');
            console.log(JSON.stringify({ byString: datasetGetRaw('128'),
                                         byNumber: datasetGetRaw(128) }));
            """)
        assert result["byString"] == "alias-owner"
        assert result["byNumber"] == "uid-owner"


# ---------------------------------------------------------------------------
# Unknown selector yields undefined (AC4 shape)
# ---------------------------------------------------------------------------


class TestUnknownSelector:
    def test_unknown_alias_is_undefined(self):
        result = _run("""
            __defineDataset(1, 'known');
            __setDatasetRaw(1, 1);
            var v = datasetGetRaw('nope');
            console.log(JSON.stringify({ missing: (v === undefined) }));
            """)
        assert result["missing"] is True

    def test_unknown_uniqueid_is_undefined(self):
        result = _run("""
            __defineDataset(1, 'known');
            var v = datasetGetFinal(999);
            console.log(JSON.stringify({ missing: (v === undefined) }));
            """)
        assert result["missing"] is True

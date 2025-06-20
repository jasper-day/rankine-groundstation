import pytest
from pytest import approx
import math
from mpcc.projection import (
    Cartesian3,
    Cartographic,
    Enu,
    ecef_to_enu,
    enu_to_ecef,
    geodetic_to_enu,
    enu_to_geodetic,
)


def test_ecef_enu_roundtrip():
    """Test ECEF <-> ENU conversion reversibility"""
    origin = Cartographic(latitude=37.7749, longitude=-122.4194, altitude=100.0)

    # Test multiple ECEF points
    test_points = [
        Cartesian3(x=-2707221.0, y=-4260847.0, z=3885494.0),
        Cartesian3(x=-2707000.0, y=-4260500.0, z=3885800.0),
        Cartesian3(x=-2708000.0, y=-4261000.0, z=3885000.0),
    ]

    for ecef_orig in test_points:
        enu = ecef_to_enu(ecef_orig, origin)
        ecef_recovered = enu_to_ecef(enu, origin)

        assert ecef_orig.x == approx(ecef_recovered.x, abs=1e-6)
        assert ecef_orig.y == approx(ecef_recovered.y, abs=1e-6)
        assert ecef_orig.z == approx(ecef_recovered.z, abs=1e-6)


def test_geodetic_enu_roundtrip():
    """Test Geodetic <-> ENU conversion reversibility"""
    origin = Cartographic(latitude=37.7749, longitude=-122.4194, altitude=100.0)

    # Test multiple geodetic points
    test_points = [
        Cartographic(latitude=37.7849, longitude=-122.4094, altitude=150.0),
        Cartographic(latitude=37.7649, longitude=-122.4294, altitude=50.0),
        Cartographic(latitude=37.7949, longitude=-122.3994, altitude=200.0),
    ]

    for geodetic_orig in test_points:
        enu = geodetic_to_enu(geodetic_orig, origin)
        geodetic_recovered = enu_to_geodetic(enu, origin)

        assert geodetic_orig.latitude == approx(geodetic_recovered.latitude, abs=1e-9)
        assert geodetic_orig.longitude == approx(geodetic_recovered.longitude, abs=1e-9)
        assert geodetic_orig.altitude == approx(geodetic_recovered.altitude, abs=1e-6)


def test_enu_roundtrip():
    """Test ENU roundtrip through both ECEF and Geodetic paths"""
    origin = Cartographic(latitude=(37.7749), longitude=(-122.4194), altitude=100.0)

    # Test multiple ENU points
    test_points = [
        Enu(e=1000.0, n=2000.0, u=50.0),
        Enu(e=-500.0, n=1500.0, u=-25.0),
        Enu(e=0.0, n=0.0, u=0.0),
    ]

    for enu_orig in test_points:
        # Round trip through ECEF
        ecef = enu_to_ecef(enu_orig, origin)
        enu_recovered = ecef_to_enu(ecef, origin)

        assert enu_orig.e == approx(enu_recovered.e, abs=1e-6)
        assert enu_orig.n == approx(enu_recovered.n, abs=1e-6)
        assert enu_orig.u == approx(enu_recovered.u, abs=1e-6)

        # Round trip through Geodetic
        geodetic = enu_to_geodetic(enu_orig, origin)
        enu_recovered2 = geodetic_to_enu(geodetic, origin)

        assert enu_orig.e == approx(enu_recovered2.e, abs=1e-6)
        assert enu_orig.n == approx(enu_recovered2.n, abs=1e-6)
        assert enu_orig.u == approx(enu_recovered2.u, abs=1e-6)


def test_origin_consistency():
    """Test that origin point converts to zero ENU"""
    origin = Cartographic(latitude=40.7128, longitude=-74.0060, altitude=10.0)

    enu = geodetic_to_enu(origin, origin)

    assert enu.e == approx(0.0, abs=1e-6)
    assert enu.n == approx(0.0, abs=1e-6)
    assert enu.u == approx(0.0, abs=1e-6)

import pytest


@pytest.fixture
def valid_numpy():
    try:
        import numpy as np

        # Stack all used numpy functions
        np.array([[1, 2, 3]])
        c1 = np.zeros((6, 1))
        c2 = np.ones((6, 1))
        np.eye(5)
        np.concatenate((c1, c2), axis=0)
        np.vstack((c1, c2))
        np.hstack((c1, c2))
        np.linalg.norm(c1)
        np.asarray([1, 2, 3])
        c1.ravel(order="F")
        np.append(c1, c2, axis=1)
        np.linspace(0, 10)
        np.cos(0)
        np.sin(0)

        return True
    except Exception as e:
        return False


@pytest.fixture
def valid_casadi():
    try:
        import casadi as ca

        # Stack all used casadi function
        Rmat = ca.MX(3, 3)
        Rzeros = ca.MX.zeros(3, 3)

        return True
    except Exception as e:
        return False


@pytest.fixture
def valid_stringio():
    try:
        import numpy as np
        from io import BytesIO as StringIO

        data_buf = StringIO()
        arr = np.array([1, 2, 3])
        data_buf.write(bytes(arr.tolist()))
        data_buf.getvalue()

        return True
    except Exception as e:
        return False


def test_numpy(valid_numpy):
    assert valid_numpy


def test_casadi(valid_casadi):
    assert valid_casadi


def test_stringio(valid_stringio):
    assert valid_stringio

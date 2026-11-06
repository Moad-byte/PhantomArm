import pytest

def test_model_compiles_if_installed():
    mujoco=pytest.importorskip('mujoco')
    from phantomarm.verify_model import main
    main()

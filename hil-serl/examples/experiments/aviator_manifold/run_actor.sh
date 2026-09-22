# AviatorManifold RLPD actor (Task 2.1).
# Mirrors examples/experiments/usb_pickup_insertion/run_actor.sh.
# HIL-SERL pinned at commit c32939b.
export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.1 && \
python ../../train_rlpd.py "$@" \
    --exp_name=aviator_manifold \
    --checkpoint_path=../../experiments/aviator_manifold/debug \
    --actor \

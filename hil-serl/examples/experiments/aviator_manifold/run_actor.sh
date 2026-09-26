# AviatorManifold RLPD actor (Task 2.3, 8-way vectorized).
# Mirrors examples/experiments/usb_pickup_insertion/run_actor.sh.
# HIL-SERL pinned at commit c32939b.
# Run from this directory (examples/experiments/aviator_manifold/).
export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.1 && \
python ../../train_rlpd.py "$@" \
    --exp_name=aviator_manifold \
    --checkpoint_path=../../experiments/aviator_manifold/debug_rlpd \
    --bc_checkpoint_path=../../experiments/aviator_manifold/debug \
    --actor \

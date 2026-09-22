# AviatorManifold RLPD learner (Task 2.1).
# Mirrors examples/experiments/usb_pickup_insertion/run_learner.sh.
# HIL-SERL pinned at commit c32939b.
export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.3 && \

# Expand the DP demo pkls into repeated --demo_path flags (abseil multi_string).
DEMO_ARGS=""
for f in ../../../data/aviator/dp_demo/traj_*.pkl; do
  DEMO_ARGS="$DEMO_ARGS --demo_path=$f"
done

python ../../train_rlpd.py "$@" \
    --exp_name=aviator_manifold \
    --checkpoint_path=../../experiments/aviator_manifold/debug \
    --learner \
    $DEMO_ARGS

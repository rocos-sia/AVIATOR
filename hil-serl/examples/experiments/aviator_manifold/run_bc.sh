# AviatorManifold BC pretraining (Task 2.2, BLOCKER #12).
# Mirrors run_learner.sh; HIL-SERL pinned at commit c32939b.
# Run from this directory (examples/experiments/aviator_manifold/).
export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.3 && \

# Expand the DP demo pkls into repeated --demo_path flags (abseil multi_string).
DEMO_ARGS=""
for f in ../../../data/aviator/dp_demo/traj_*.pkl; do
  DEMO_ARGS="$DEMO_ARGS --demo_path=$f"
done

python ../../train_bc.py "$@" \
    --exp_name=aviator_manifold \
    --bc_checkpoint_path=../../experiments/aviator_manifold/debug \
    $DEMO_ARGS

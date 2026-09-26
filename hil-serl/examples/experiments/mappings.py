from experiments.aviator_manifold.config import TrainConfig as AviatorTrainConfig

CONFIG_MAPPING = {
                "aviator_manifold": AviatorTrainConfig,
               }

# Real-robot experiments depend on franka_env (serl_robot_infra), which is not
# installed in the offline serl_clean env used for proprio-only training.
# Register them only when the dependency is importable.
try:
    from experiments.ram_insertion.config import TrainConfig as RAMInsertionTrainConfig
    from experiments.usb_pickup_insertion.config import TrainConfig as USBPickupInsertionTrainConfig
    from experiments.object_handover.config import TrainConfig as ObjectHandoverTrainConfig
    from experiments.egg_flip.config import TrainConfig as EggFlipTrainConfig

    CONFIG_MAPPING.update({
        "ram_insertion": RAMInsertionTrainConfig,
        "usb_pickup_insertion": USBPickupInsertionTrainConfig,
        "object_handover": ObjectHandoverTrainConfig,
        "egg_flip": EggFlipTrainConfig,
    })
except ImportError:
    pass

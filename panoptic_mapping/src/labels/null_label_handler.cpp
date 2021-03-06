#include "panoptic_mapping/labels/null_label_handler.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<LabelHandlerBase, NullLabelHandler>
    NullLabelHandler::registration_("null");

void NullLabelHandler::Config::setupParamsAndPrinting() {
  printText("The null label handler does not read any labels.");
}

NullLabelHandler::NullLabelHandler(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

}  // namespace panoptic_mapping

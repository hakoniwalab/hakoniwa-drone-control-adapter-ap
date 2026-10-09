// SPDX-License-Identifier: GPL-3.0-or-later

#include "ardupilot_fixture.hpp"
#include "test/contract/cases/backend_contract_cases.hpp"
#include "test/contract/cases/ekf_contract_cases.hpp"
#include "test/contract/checker/contract_checker.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#ifndef HAKO_CONTRACT_AP_CAPABILITY
#error HAKO_CONTRACT_AP_CAPABILITY must be defined
#endif
#ifndef HAKO_CONTRACT_REPORT_DIR
#error HAKO_CONTRACT_REPORT_DIR must be defined
#endif
#ifndef HAKO_CONTRACT_PRO_COMMIT
#define HAKO_CONTRACT_PRO_COMMIT "unknown"
#endif
#ifndef HAKO_CONTRACT_AP_COMMIT
#define HAKO_CONTRACT_AP_COMMIT "unknown"
#endif

namespace contract = hako::control_link::contract;

int main()
{
    try {
        const contract::ArduPilotFixture fixture;
        auto checker = contract::ContractChecker::load(HAKO_CONTRACT_AP_CAPABILITY);
        auto raw = contract::run_backend_contract_cases(
            fixture, checker.capability_value("torque_unit"));
        auto ekf = contract::run_ekf_contract_cases(fixture);
        raw.insert(raw.end(), ekf.begin(), ekf.end());

        std::vector<contract::CheckedTestResult> checked;
        bool passed = true;
        for (const auto& result : raw) {
            auto item = checker.evaluate(result);
            std::cout << '[' << contract::to_string(item.final_result) << "] "
                      << fixture.name() << ' ' << result.test_id
                      << " raw=" << contract::to_string(result.raw_result) << '\n';
            passed &= item.final_result != contract::FinalResult::Fail;
            checked.push_back(std::move(item));
        }

        const auto report = checker.make_report(
            checked, HAKO_CONTRACT_AP_CAPABILITY,
            HAKO_CONTRACT_PRO_COMMIT, HAKO_CONTRACT_AP_COMMIT);
        const std::filesystem::path report_dir{HAKO_CONTRACT_REPORT_DIR};
        std::filesystem::create_directories(report_dir);
        const auto report_path = report_dir / "ardupilot.json";
        std::ofstream output(report_path);
        if (!output) throw std::runtime_error("cannot write contract report");
        output << report.dump(2) << '\n';
        std::cout << "Report: " << report_path << '\n';
        return passed ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Contract test error: " << error.what() << '\n';
        return 2;
    }
}

#!/usr/bin/env groovy

def pipelines = ['gcc', 'clang', 'asan', 'ubsan', 'tsan']
def branches = [:]

for ( pipeline in pipelines ) {
  def branchName = pipeline

  branches[branchName] = {
    node('delphyne-linux-bionic-unprovisioned') {
      timeout(60) {
        ansiColor('xterm') {
          try {
            stage('[' + branchName + ']' + 'checkout') {
              dir('src/maliput_multilane') {
                checkout scm
              }
            }
            stage('[' + branchName + ']' + 'checkout_index') {
              sh 'src/maliput_multilane/ci/jenkins/checkout_index'
            }
            withEnv(['COLCON_BUILD_EXTRA_ARGS=--packages-up-to maliput_multilane',
                    'COLCON_TEST_EXTRA_ARGS=--packages-up-to maliput_multilane']) {
              load './index/ci/jenkins/pipeline_' + branchName + '.groovy'
            }
          } finally {
            cleanWs(notFailBuild: true)
          }
        }
      }
    }
  }
}
branches.failFast = true
// Give the branches to Jenkins for parallel execution:
parallel branches

#!/usr/bin/env groovy
//
// Multibranch pipeline: builds ArduCopter for the CubeOrangePlus inside the
// container described by the repository Dockerfile.
//
// Agent requirements:
//   * docker CLI usable by the Jenkins user (agent label "docker" below)
//   * git
//
// Only core pipeline steps are used (checkout / sh / archiveArtifacts), so the
// Docker Pipeline plugin is not needed.

pipeline {
    agent { label 'docker' }

    parameters {
        booleanParam(name: 'CLEAN_BUILD',
                     defaultValue: false,
                     description: 'Run "waf distclean" before configuring.')
        booleanParam(name: 'REBUILD_IMAGE',
                     defaultValue: false,
                     description: 'Rebuild the build container ignoring the docker layer cache.')
        string(name: 'BASE_TAG',
               defaultValue: '22.04',
               description: 'Ubuntu tag used as the Dockerfile base image.')
    }

    options {
        timestamps()
        timeout(time: 3, unit: 'HOURS')
        buildDiscarder(logRotator(numToKeepStr: '20', artifactNumToKeepStr: '10'))
        disableConcurrentBuilds()
        skipDefaultCheckout(true)
    }

    environment {
        // Tagged per branch so branches of the multibranch job never clobber each other.
        BRANCH_SLUG  = "${env.BRANCH_NAME.replaceAll('[^A-Za-z0-9_.-]', '-')}"
        IMAGE_TAG    = "ardupilot-build:${env.BRANCH_NAME.replaceAll('[^A-Za-z0-9_.-]', '-').toLowerCase()}"
        CONTAINER_WS = '/ardupilot'
        BOARD        = 'CubeOrangePlus'
        // Nexus coordinates for the published firmware.
        NEXUS_URL        = 'nexus.aeronavics.com'
        NEXUS_REPO       = 'release_library'
        NEXUS_GROUP      = 'arducopter'
        NEXUS_CREDS      = 'JenkinsAdmin'
        // Shared release library on the build agent (note the space in the path).
        RELEASE_LIB_DIR  = '/home/releaseLibrary/AC-16/Flight Controller'
        // ccache lives in the per-branch @tmp dir: it survives workspace cleaning and is
        // owned by the Jenkins user (a docker named volume would be created root owned).
        CCACHE_HOST_DIR = "${env.WORKSPACE_TMP ?: env.WORKSPACE + '@tmp'}/ccache"
    }

    stages {
        stage('Checkout') {
            steps {
                // Plain "checkout scm" only: reading scm.branches / scm.extensions /
                // scm.browser to inject a SubmoduleOption trips the Groovy sandbox
                // ("Scripts not permitted to use method hudson.scm.SCM getBrowser").
                // The submodules are pulled in below instead -- every URL in
                // .gitmodules is public https, so no Jenkins credentials are needed.
                checkout scm
                sh '''
                    set -eux
                    git submodule sync --recursive
                    git submodule update --init --recursive
                '''
                script {
                    env.GIT_SHORT = sh(script: 'git rev-parse --short HEAD', returnStdout: true).trim()
                    env.HOST_UID  = sh(script: 'id -u', returnStdout: true).trim()
                    env.HOST_GID  = sh(script: 'id -g', returnStdout: true).trim()

                    // Version for Nexus comes from the branch name, e.g.
                    // "ANVCopter-4.6.3" -> "4.6.3". Done with grep rather than a
                    // Groovy "=~" because java.util.regex.Matcher is not
                    // CPS-serialisable and needs script approval in the sandbox.
                    env.FW_VERSION = sh(
                        script: 'echo "${BRANCH_NAME}" | grep -oE "[0-9]+\\.[0-9]+(\\.[0-9]+)*" | head -n1 || true',
                        returnStdout: true).trim()
                }
                echo "Building ${env.BRANCH_NAME} @ ${env.GIT_SHORT}"
                script {
                    if (env.FW_VERSION) {
                        echo "Firmware version parsed from branch name: ${env.FW_VERSION}"
                    } else {
                        echo "No version number in branch name '${env.BRANCH_NAME}' - " +
                             'the build will run but nothing will be published to Nexus.'
                    }
                }
            }
        }

        stage('Build container image') {
            steps {
                // The container user is given the Jenkins user's uid/gid so the bind
                // mounted workspace stays writable and artifacts are not left root owned.
                sh '''
                    set -eux
                    CACHE_ARGS=""
                    if [ "${REBUILD_IMAGE}" = "true" ]; then
                        CACHE_ARGS="--no-cache --pull"
                    fi
                    docker build ${CACHE_ARGS} \
                        --build-arg TAG="${BASE_TAG}" \
                        --build-arg USER_UID="${HOST_UID}" \
                        --build-arg USER_GID="${HOST_GID}" \
                        -t "${IMAGE_TAG}" \
                        -f Dockerfile .
                '''
            }
        }

        stage('Build ArduCopter') {
            steps {
                sh 'mkdir -p "${CCACHE_HOST_DIR}"'
                script {
                    def cmds = params.CLEAN_BUILD ? ['./waf distclean'] : []
                    cmds += [
                        "./waf configure --board=${env.BOARD}",
                        './waf copter',
                        'ccache -s',
                    ]
                    runInContainer(cmds.join('\n'))
                }
            }
        }

        stage('Collect firmware') {
            steps {
                script {
                    // Workspace relative path, reused verbatim by the Nexus upload.
                    env.APJ_FILE = "artifacts/${env.BOARD}-${env.BRANCH_SLUG}-${env.GIT_SHORT}-arducopter.apj"
                }
                sh '''
                    set -eux
                    rm -rf artifacts && mkdir -p artifacts
                    cp -a "build/${BOARD}/bin/arducopter.apj" "${APJ_FILE}"
                    ls -l artifacts
                '''
                archiveArtifacts artifacts: 'artifacts/*.apj', fingerprint: true
            }
        }

        stage('Publish to Nexus') {
            when {
                // Feature branches without a version in their name still build and
                // archive, they just do not publish.
                expression { return env.FW_VERSION != null && env.FW_VERSION != '' }
            }
            steps {
                nexusArtifactUploader(
                    nexusVersion: 'nexus3',
                    protocol: 'https',
                    nexusUrl: env.NEXUS_URL,          // host only, no scheme
                    repository: env.NEXUS_REPO,
                    credentialsId: env.NEXUS_CREDS,
                    groupId: env.NEXUS_GROUP,
                    version: env.FW_VERSION,
                    artifacts: [
                        [artifactId: env.BOARD,
                         classifier: 'arducopter',
                         type: 'apj',
                         file: env.APJ_FILE]
                    ]
                )
                echo "Uploaded ${env.APJ_FILE} to ${env.NEXUS_URL}/${env.NEXUS_REPO} " +
                     "as ${env.NEXUS_GROUP}:${env.BOARD}:${env.FW_VERSION}:arducopter:apj"
            }
        }

        stage('Copy to release library') {
            when {
                expression { return env.FW_VERSION != null && env.FW_VERSION != '' }
            }
            steps {
                sh '''
                    set -eu
                    dest="${RELEASE_LIB_DIR}/arducopter-${FW_VERSION}.apj"

                    # Deliberately not "mkdir -p": the release library is a shared
                    # location, and silently creating it would scatter releases across
                    # agents that happen not to have it mounted.
                    if [ ! -d "${RELEASE_LIB_DIR}" ]; then
                        echo "Release library '${RELEASE_LIB_DIR}' not present on $(hostname)." >&2
                        echo "Is this the right agent, and is the share mounted?" >&2
                        exit 1
                    fi

                    cp -f "${APJ_FILE}" "$dest"
                    ls -l "$dest"
                '''
            }
        }
    }

    post {
        cleanup {
            // The workspace is deliberately kept: the git checkout, submodules and the
            // waf build/ tree make subsequent builds of this branch incremental.
            sh 'rm -f .jenkins-step.sh'
        }
    }
}

// Run a shell snippet inside the build container.
//
// The image ENTRYPOINT sources ~/.ardupilot_env (arm-none-eabi toolchain and the
// ccache shims), so the commands are passed as the container's arguments; using
// "docker exec" instead would bypass the entrypoint and lose the toolchain PATH.
void runInContainer(String script) {
    writeFile file: '.jenkins-step.sh', text: "#!/bin/bash\nset -eux\n${script}\n"
    sh '''
        set -eu
        chmod +x .jenkins-step.sh
        docker run --rm \
            -u "${HOST_UID}:${HOST_GID}" \
            -e HOME=/home/ardupilot \
            -e CCACHE_DIR=/ccache \
            -e CCACHE_MAXSIZE=2G \
            -e GIT_CONFIG_COUNT=1 \
            -e GIT_CONFIG_KEY_0=safe.directory \
            -e GIT_CONFIG_VALUE_0="*" \
            -v "${WORKSPACE}:${CONTAINER_WS}" \
            -v "${CCACHE_HOST_DIR}:/ccache" \
            -w "${CONTAINER_WS}" \
            "${IMAGE_TAG}" \
            ./.jenkins-step.sh
    '''
}

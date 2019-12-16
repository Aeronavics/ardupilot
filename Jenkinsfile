pipeline {
    agent none
    stages {
        stage('Firmware Release') {
            agent {
                docker{
                    registryUrl 'http://pelardon.aeronavics.com:8083'
                    registryCredentialsId 'aeronavics_registry_user'
                    image 'pelardon.aeronavics.com:8083/ardupilot_container:latest'
                    args '-u root'
                }
            }
            steps {
                sh 'git fetch --tags'
                sh 'whoami'
                sh 'ls -al'
                sh 'echo $PATH'
                sh 'pip list'
                sh './waf configure --board=CubeBlack'
                sh './waf copter'
            }
            post {
                success {
                    stash(includes: 'build/**/*.apj', name: 'firmware')
                    archiveArtifacts artifacts: 'build/**/*.apj'
                }
                cleanup {
                    sh 'rm -r ${WORKSPACE}/build || true'
                    sh 'git clean -ff -x -d .'
                }
            }
        }
        stage('deploy stable') {
            agent {
                node{
                    label "linux"
                }
            }
            environment {
                QGC_REGISTRY_CREDS = credentials('qgc_uploader')
            }
            when {
                branch 'prod'
            }
            steps {
                script {
                    env.QGC_CONFIG = 'release'
                    env.VERSION_NAME = getVersion()
                    env.HASH_NAME = getHash()
                }

                unstash 'firmware'
                nexusArtifactUploader(credentialsId: 'qgc_uploader', groupId: 'prod', nexusUrl: 'pelardon.aeronavics.com:8086/nexus', nexusVersion: 'nexus3', protocol: 'http', repository: 'flight_controller_firmware', version: "${env.VERSION_NAME}", artifacts: [
                        [artifactId: "${env.VERSION_NAME}", classifier: "${env.HASH_NAME}", file: 'build/CubeBlack/bin/arducopter.apj', type: 'apj'],
                ])
            }
        }
    }
}

def getTag()
{
    tags = sh(returnStdout: true, script: "git tag").trim()
    print tags
    return tags
}

def getVersion()
{
    tags = sh(returnStdout: true, script: "git describe --tags --abbrev=7").trim()
    print tags
    return tags
}

def getHash()
{
    hash = sh(returnStdout: true, script: "git rev-parse --short HEAD").trim()
    print hash
    return hash
}

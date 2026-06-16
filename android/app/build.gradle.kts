plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.mythcal.mec"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.mythcal.mec"
        minSdk = 26          // Camera2 full + MediaPipe Tasks
        targetSdk = 34
        versionCode = 2
        versionName = "1.0.0"

        ndk { abiFilters += listOf("arm64-v8a") } // most phones; add others as needed
        externalNativeBuild {
            cmake { cppFlags += "-std=c++17" }
        }
    }

    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }

    // The MediaPipe .task model is a downloaded asset; don't compress it.
    androidResources { noCompress += "task" }
}

dependencies {
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.appcompat:appcompat:1.7.0")
    // MediaPipe Pose Landmarker (Tasks Vision).
    implementation("com.google.mediapipe:tasks-vision:0.10.14")
}

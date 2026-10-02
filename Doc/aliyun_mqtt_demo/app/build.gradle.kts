

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
}

val localProps = java.util.Properties().apply {
    val f = rootProject.file("local.properties")
    if (f.exists()) {
        f.reader(Charsets.UTF_8).use { load(it) }
    }
}


android {
    namespace = "com.example.myapplication"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.example.myapplication"
        minSdk = 22
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        buildConfigField("String", "PRODUCT_KEY", "\"${localProps.getProperty("productKey") ?: ""}\"")
        buildConfigField("String", "DEVICE_NAME", "\"${localProps.getProperty("deviceName") ?: ""}\"")
        buildConfigField("String", "DEVICE_SECRET", "\"${localProps.getProperty("deviceSecret") ?: ""}\"")
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
            signingConfig = signingConfigs.getByName("debug")
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
        jvmTarget = "1.8"
    }
    buildFeatures {
        buildConfig = true
    }
}

dependencies {
    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.material)
    implementation(libs.androidx.activity)
    implementation(libs.androidx.constraintlayout)
    implementation(files("mysql-connector-java-5.1.49-bin.jar"))
    implementation (libs.com.google.android.material.material.v140.x2)
//    implementation(libs.material.v100)
    implementation (libs.org.eclipse.paho.client.mqttv3)
    implementation(libs.firebase.firestore)
    implementation(libs.gson)//    implementation 'com.google.android.material:material:1.0.0'
    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)


}
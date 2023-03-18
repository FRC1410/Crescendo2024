plugins {
	`kotlin-dsl`
}

repositories {
	mavenCentral()
}

tasks {
	compileKotlin {
		val version = JavaVersion.VERSION_17.majorVersion
		targetCompatibility = version
		sourceCompatibility = version
	}
}

# Local copy of espressif/usb 1.1.0 (overrides managed_components/espressif__usb)

Patched for Mini-FT8: `usb_host_claim_ep_filter()` (weak, default true) lets the
application skip endpoints when claiming an interface. The FX-4CR has two hubs
and three devices behind them, which leaves only one free channel on the
ESP32-S3's 8 host channels; the CAT bridge (CH340) is claimed with only its
bulk OUT endpoint. See main/stream_uac.cpp (fx_ch340_write).

## Build notes

- `main/idf_component.yml` points `espressif/usb` here with `override_path`.
- `dependencies.lock` records this component as `path: /project/components/espressif__usb`
  (the Docker build mount point). With that exact path the component manager keeps
  the locked versions of the other components (usb_host_cdc_acm 2.2.0,
  usb_host_uac 1.3.3, tinyusb 0.19.0~3). Building from another directory makes it
  re-solve and pick newer versions; restore the lock afterwards if that matters.
